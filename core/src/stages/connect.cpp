/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner, Robert Haschke
   Desc:    Connect arbitrary states by motion planning
*/

#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/merge.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <fmt/core.h>
#include <fmt/ostream.h>

#if FMT_VERSION >= 90000
template <>
struct fmt::formatter<Eigen::Transpose<Eigen::Map<const Eigen::Matrix<double, -1, 1>, 0, Eigen::Stride<0, 0>>>>
  : fmt::ostream_formatter
{};
#endif

using namespace trajectory_processing;

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Connect");

Connect::Connect(const std::string& name, const GroupPlannerVector& planners) : Connecting(name), planner_(planners) {
	setTimeout(1.0);
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	p.declare<MergeMode>("merge_mode", WAYPOINTS, "merge mode");
	p.declare<double>("max_distance", 1e-2, "maximally accepted distance between end and goal sate");
	p.declare<moveit_msgs::msg::Constraints>("path_constraints", moveit_msgs::msg::Constraints(),
	                                         "constraints to maintain during trajectory");
	properties().declare<TimeParameterizationPtr>("merge_time_parameterization",
	                                              std::make_shared<TimeOptimalTrajectoryGeneration>());
}

void Connect::reset() {
	Connecting::reset();
	merged_jmg_.reset();
	subsolutions_.clear();
	states_.clear();
}

void Connect::init(const core::RobotModelConstPtr& robot_model) {
	Connecting::init(robot_model);

	InitStageException errors;
	if (planner_.empty())
		errors.push_back(*this, "empty set of groups");

	std::vector<const moveit::core::JointModelGroup*> groups;
	for (const GroupPlannerVector::value_type& pair : planner_) {
		if (!robot_model->hasJointModelGroup(pair.first))
			errors.push_back(*this, "invalid group: " + pair.first);
		else if (!pair.second)
			errors.push_back(*this, "invalid planner for group: " + pair.first);
		else {
			pair.second->init(robot_model);

			auto jmg = robot_model->getJointModelGroup(pair.first);
			groups.push_back(jmg);
		}
	}

	if (!errors && groups.size() >= 2 && !merged_jmg_) {  // enable merging?
		try {
			merged_jmg_.reset(task_constructor::merge(groups));
		} catch (const std::runtime_error& e) {
			RCLCPP_INFO_STREAM(LOGGER, fmt::format("{}: {}. Disabling merging.", this->name(), e.what()));
		}
	}

	if (errors)
		throw errors;
}

bool Connect::compatible(const InterfaceState& from_state, const InterfaceState& to_state) const {
	if (!Connecting::compatible(from_state, to_state))
		return false;

	const moveit::core::RobotState& from = from_state.scene()->getCurrentState();
	const moveit::core::RobotState& to = to_state.scene()->getCurrentState();

	// compose set of joint names we plan for
	std::set<std::string> planned_joint_names;
	for (const GroupPlannerVector::value_type& pair : planner_) {
		const moveit::core::JointModelGroup* jmg = from.getJointModelGroup(pair.first);
		const auto& names = jmg->getJointModelNames();
		planned_joint_names.insert(names.begin(), names.end());
	}
	// all active joints that we don't plan for should match
	for (const moveit::core::JointModel* jm : from.getRobotModel()->getJointModels()) {
		if (planned_joint_names.count(jm->getName()))
			continue;  // ignore joints we plan for

		const unsigned int num = jm->getVariableCount();
		Eigen::Map<const Eigen::VectorXd> positions_from(from.getJointPositions(jm), num);
		Eigen::Map<const Eigen::VectorXd> positions_to(to.getJointPositions(jm), num);
		if (!(positions_from - positions_to).isZero(1e-4)) {
			RCLCPP_INFO_STREAM(LOGGER, fmt::format("Deviation in joint {}: [{}] != [{}]", jm->getName(),
			                                       positions_from.transpose(), positions_to.transpose()));
			return false;
		}
	}
	return true;
}

void Connect::compute(const InterfaceState& from, const InterfaceState& to) {
	const auto& props = properties();
	double timeout = this->timeout();
	MergeMode mode = props.get<MergeMode>("merge_mode");
	double max_distance = props.get<double>("max_distance");
	const auto& path_constraints = props.get<moveit_msgs::msg::Constraints>("path_constraints");

	const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
	std::vector<PlannerIdTrajectoryPair> sub_trajectories;

	std::vector<planning_scene::PlanningSceneConstPtr> intermediate_scenes;
	planning_scene::PlanningSceneConstPtr start = from.scene();
	intermediate_scenes.push_back(start);

	bool success = false;
	std::string comment = "No planners specified";
	std::vector<double> positions;
	for (const GroupPlannerVector::value_type& pair : planner_) {
		// set intermediate goal state
		planning_scene::PlanningScenePtr end = start->diff();
		const moveit::core::JointModelGroup* jmg = final_goal_state.getJointModelGroup(pair.first);
		final_goal_state.copyJointGroupPositions(jmg, positions);
		moveit::core::RobotState& goal_state = end->getCurrentStateNonConst();
		goal_state.setJointGroupPositions(jmg, positions);
		goal_state.update();
		intermediate_scenes.push_back(end);

		robot_trajectory::RobotTrajectoryPtr trajectory;
		auto result = pair.second->plan(start, end, jmg, timeout, trajectory, path_constraints);
		success = bool(result);
		sub_trajectories.push_back({ pair.second->getPlannerId(), trajectory });

		if (!success) {
			comment = result.message;
			break;
		}

		if (trajectory->getLastWayPoint().distance(goal_state, jmg) > max_distance) {
			success = false;
			comment = "Trajectory end-point deviates too much from goal state";
			break;
		}

		// continue from reached state
		start = end;
	}

	SolutionBasePtr solution;
	if (success && mode != SEQUENTIAL)  // try to merge
		solution = merge(sub_trajectories, intermediate_scenes, from.scene()->getCurrentState());
	if (!solution)  // success == false or merging failed: store sequentially
		solution = makeSequential(sub_trajectories, intermediate_scenes, from, to);
	if (!success)  // error during sequential planning
		solution->markAsFailure(comment);
	connect(from, to, solution);
}

SolutionSequencePtr
Connect::makeSequential(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                        const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                        const InterfaceState& from, const InterfaceState& to) {
	assert(!sub_trajectories.empty());
	assert(sub_trajectories.size() + 1 == intermediate_scenes.size());

	/* We need to decouple the sequence of subsolutions, created here, from the external from and to states.
	   Hence, we create new interface states for all subsolutions. */
	const InterfaceState* start = &*states_.insert(states_.end(), InterfaceState(from.scene()));

	auto scene_it = intermediate_scenes.begin();
	SolutionSequence::container_type sub_solutions;
	for (const auto& sub : sub_trajectories) {
		// persistently store sub solution
		auto inserted = subsolutions_.insert(subsolutions_.end(),
		                                     SubTrajectory(sub.trajectory, 0.0, std::string(""), sub.planner_id));
		inserted->setCreator(this);
		if (!sub.trajectory)  // a null RobotTrajectoryPtr indicates a failure
			inserted->markAsFailure();
		// push back solution pointer
		sub_solutions.push_back(&*inserted);

		// create a new end state, either from intermediate or final planning scene
		planning_scene::PlanningSceneConstPtr end_ps =
		    (sub_solutions.size() < sub_trajectories.size()) ? *++scene_it : to.scene();
		const InterfaceState* end = &*states_.insert(states_.end(), InterfaceState(end_ps));

		// provide newly created start/end states
		subsolutions_.back().setStartState(*start);
		subsolutions_.back().setEndState(*end);

		start = end;  // end state becomes next start state
	}

	return std::make_shared<SolutionSequence>(std::move(sub_solutions));
}

SubTrajectoryPtr Connect::merge(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
                                const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                const moveit::core::RobotState& state) {
	// no need to merge if there is only a single sub trajectory
	if (sub_trajectories.size() == 1)
		return std::make_shared<SubTrajectory>(sub_trajectories.at(0).trajectory, 0.0, std::string(""),
		                                       sub_trajectories.at(0).planner_id);

	// split sub_trajectories into trajectories and joined planner_ids
	std::string planner_ids;
	std::vector<robot_trajectory::RobotTrajectoryConstPtr> subs;
	subs.reserve(sub_trajectories.size());
	for (auto it = sub_trajectories.begin(); it != sub_trajectories.end(); ++it) {
		subs.push_back(it->trajectory);
		if (it != sub_trajectories.begin())
			planner_ids += ", ";
		planner_ids += it->planner_id;
	}

	auto jmg = merged_jmg_.get();
	assert(jmg);
	auto timing = properties().get<TimeParameterizationPtr>("merge_time_parameterization");
	robot_trajectory::RobotTrajectoryPtr trajectory = task_constructor::merge(subs, state, jmg, *timing);
	if (!trajectory)
		return SubTrajectoryPtr();

	// check merged trajectory for collisions
	if (!intermediate_scenes.front()->isPathValid(*trajectory,
	                                              properties().get<moveit_msgs::msg::Constraints>("path_constraints")))
		return SubTrajectoryPtr();

	return std::make_shared<SubTrajectory>(trajectory, 0.0, std::string(""), planner_ids);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
