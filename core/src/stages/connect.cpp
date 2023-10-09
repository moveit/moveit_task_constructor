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
			RCLCPP_INFO_STREAM(LOGGER, this->name() << ": " << e.what() << ". Disabling merging.");
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
			RCLCPP_INFO_STREAM(LOGGER, "Deviation in joint " << jm->getName() << ": [" << positions_from.transpose()
			                                                 << "] != [" << positions_to.transpose() << "]");
			return false;
		}
	}
	return true;
}

void Connect::compute(const InterfaceState& from, const InterfaceState& to) {
	const auto& props = properties();
	double timeout = this->timeout();
	MergeMode mode = props.get<MergeMode>("merge_mode");
	const auto& path_constraints = props.get<moveit_msgs::msg::Constraints>("path_constraints");

	const moveit::core::RobotState& final_goal_state = to.scene()->getCurrentState();
	std::vector<PlannerIdTrajectoryPair> trajectory_planner_vector;

	std::vector<planning_scene::PlanningSceneConstPtr> intermediate_scenes;
	planning_scene::PlanningSceneConstPtr start = from.scene();
	intermediate_scenes.push_back(start);

	bool success = false;
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
		success = pair.second->plan(start, end, jmg, timeout, trajectory, path_constraints);
		trajectory_planner_vector.push_back(PlannerIdTrajectoryPair({ pair.second->getPlannerId(), trajectory }));

		if (!success)
			break;

		// continue from reached state
		start = end;
	}

	SolutionBasePtr solution;
	if (success && mode != SEQUENTIAL)  // try to merge
		solution = merge(trajectory_planner_vector, intermediate_scenes, from.scene()->getCurrentState());
	if (!solution)  // success == false or merging failed: store sequentially
		solution = makeSequential(trajectory_planner_vector, intermediate_scenes, from, to);
	if (!success)  // error during sequential planning
		solution->markAsFailure();

	connect(from, to, solution);
}

SolutionSequencePtr
Connect::makeSequential(const std::vector<PlannerIdTrajectoryPair>& trajectory_planner_vector,
                        const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                        const InterfaceState& from, const InterfaceState& to) {
	assert(!trajectory_planner_vector.empty());
	assert(trajectory_planner_vector.size() + 1 == intermediate_scenes.size());

	/* We need to decouple the sequence of subsolutions, created here, from the external from and to states.
	   Hence, we create new interface states for all subsolutions. */
	const InterfaceState* start = &*states_.insert(states_.end(), InterfaceState(from.scene()));

	auto scene_it = intermediate_scenes.begin();
	SolutionSequence::container_type sub_solutions;
	for (const auto& pair : trajectory_planner_vector) {
		// persistently store sub solution
		auto inserted = subsolutions_.insert(
		    subsolutions_.end(), SubTrajectory(pair.robot_trajectory_ptr, 0.0, std::string(""), pair.planner_name));
		inserted->setCreator(this);
		if (!pair.robot_trajectory_ptr)  // a null RobotTrajectoryPtr indicates a failure
			inserted->markAsFailure();
		// push back solution pointer
		sub_solutions.push_back(&*inserted);

		// create a new end state, either from intermediate or final planning scene
		planning_scene::PlanningSceneConstPtr end_ps =
		    (sub_solutions.size() < trajectory_planner_vector.size()) ? *++scene_it : to.scene();
		const InterfaceState* end = &*states_.insert(states_.end(), InterfaceState(end_ps));

		// provide newly created start/end states
		subsolutions_.back().setStartState(*start);
		subsolutions_.back().setEndState(*end);

		start = end;  // end state becomes next start state
	}

	return std::make_shared<SolutionSequence>(std::move(sub_solutions));
}

SubTrajectoryPtr Connect::merge(const std::vector<PlannerIdTrajectoryPair>& trajectory_planner_vector,
                                const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
                                const moveit::core::RobotState& state) {
	// no need to merge if there is only a single sub trajectory
	if (trajectory_planner_vector.size() == 1)
		return std::make_shared<SubTrajectory>(trajectory_planner_vector.at(0).robot_trajectory_ptr, 0.0, std::string(""),
		                                       trajectory_planner_vector.at(0).planner_name);

	auto jmg = merged_jmg_.get();
	assert(jmg);
	auto timing = properties().get<TimeParameterizationPtr>("merge_time_parameterization");
	robot_trajectory::RobotTrajectoryPtr merged_trajectory = task_constructor::merge(
	    [&]() {
		    // Move trajectories into single vector
		    std::vector<robot_trajectory::RobotTrajectoryConstPtr> robot_traj_vector;
		    robot_traj_vector.reserve(trajectory_planner_vector.size());

		    // Copy second elements of robot planner vector into separate vector
		    std::transform(begin(trajectory_planner_vector), end(trajectory_planner_vector),
		                   std::back_inserter(robot_traj_vector),
		                   [](auto const& pair) { return pair.robot_trajectory_ptr; });
		    return robot_traj_vector;
	    }(),
	    state, jmg, *timing);

	// check merged trajectory is empty or has collisions
	if (!merged_trajectory ||
	    !intermediate_scenes.front()->isPathValid(*merged_trajectory,
	                                              properties().get<moveit_msgs::msg::Constraints>("path_constraints"))) {
		return SubTrajectoryPtr();
	}

	// Copy first elements of robot planner vector into separate vector
	std::vector<std::string> planner_names;
	planner_names.reserve(trajectory_planner_vector.size());
	std::transform(begin(trajectory_planner_vector), end(trajectory_planner_vector), std::back_inserter(planner_names),
	               [](auto const& pair) { return pair.planner_name; });

	// Create a sequence of planner names
	std::string planner_name_sequence;
	for (auto const& name : planner_names) {
		planner_name_sequence += std::string(", " + name);
	}
	return std::make_shared<SubTrajectory>(merged_trajectory, 0.0, std::string(""), planner_name_sequence);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
