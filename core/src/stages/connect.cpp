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
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

Connect::Connect(const std::string& name, const GroupPlannerVector& planners)
   : Connecting(name)
   , planner_(planners)
{
	auto& p = properties();
	p.declare<double>("timeout", 10.0, "planning timeout");
	p.declare<std::string>("group", "name of planning group");
	p.declare<moveit_msgs::Constraints>("path_constraints", moveit_msgs::Constraints(),
	                                    "constraints to maintain during trajectory");
}

void Connect::reset()
{
	Connecting::reset();
	merged_jmg_.reset();
	solutions_.clear();
	subsolutions_.clear();
	states_.clear();
}

void Connect::init(const core::RobotModelConstPtr& robot_model)
{
	Connecting::init(robot_model);

	InitStageException errors;
	if (planner_.empty())
		errors.push_back(*this, "empty set of groups");

	size_t num_joints = 0;
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
			num_joints += jmg->getJointModels().size();
		}
	}

	if (!errors && groups.size() >= 2) {  // enable merging?
		merged_jmg_.reset(task_constructor::merge(groups));
		if (merged_jmg_->getJointModels().size() != num_joints) {
			// overlapping joint groups: analyse in more detail
			std::vector<const moveit::core::JointModel*> duplicates;
			std::string names;
			if (findDuplicates(groups, merged_jmg_->getJointModels(), duplicates, names)) {
				ROS_INFO_STREAM_NAMED("Connect", this->name() << ": overlapping joint groups: " << names << ". Disabling merging.");
				merged_jmg_.reset();  // fallback to serial connect
			}
		}
	}

	if (errors)
		throw errors;
}

bool Connect::compatible(const InterfaceState& from_state, const InterfaceState& to_state) const
{
	if (!Connecting::compatible(from_state, to_state))
		return false;

	const moveit::core::RobotState& from = from_state.scene()->getCurrentState();
	const moveit::core::RobotState& to = to_state.scene()->getCurrentState();

	// compose set of joint names we plan for
	std::set<std::string> planned_joint_names;
	for (const GroupPlannerVector::value_type& pair : planner_) {
		const moveit::core::JointModelGroup *jmg = from.getJointModelGroup(pair.first);
		const auto &names = jmg->getJointModelNames();
		planned_joint_names.insert(names.begin(), names.end());
	}
	// all active joints that we don't plan for should match
	for (const moveit::core::JointModel* jm : from.getRobotModel()->getJointModels()) {
		if (planned_joint_names.count(jm->getName()))
			continue;  // ignore joints we plan for

		const unsigned int num = jm->getVariableCount();
		Eigen::Map<const Eigen::VectorXd> positions_from (from.getJointPositions(jm), num);
		Eigen::Map<const Eigen::VectorXd> positions_to (to.getJointPositions(jm), num);
		if (!positions_from.array().isApprox(positions_to.array())) {
			ROS_INFO_STREAM_ONCE_NAMED("Connect", "Deviation in joint " << jm->getName()
			                            << ": [" << positions_from.transpose()
			                            << "] != [" << positions_to.transpose() << "]");
			return false;
		}
	}
	return true;
}

bool Connect::compute(const InterfaceState &from, const InterfaceState &to) {
	const auto& props = properties();
	double timeout = props.get<double>("timeout");
	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");

	std::vector<robot_trajectory::RobotTrajectoryConstPtr> sub_trajectories;
	planning_scene::PlanningScenePtr start = from.scene()->diff();
	const moveit::core::RobotState& goal_state = to.scene()->getCurrentState();

	std::vector<planning_scene::PlanningScenePtr> intermediate_scenes;
	intermediate_scenes.push_back(start);

	std::vector<double> positions;
	for (const GroupPlannerVector::value_type& pair : planner_) {
		// set intermediate goal state
		planning_scene::PlanningScenePtr end = start->diff();
		intermediate_scenes.push_back(end);
		const moveit::core::JointModelGroup *jmg = goal_state.getJointModelGroup(pair.first);
		goal_state.copyJointGroupPositions(jmg, positions);
		end->getCurrentStateNonConst().setJointGroupPositions(jmg, positions);

		robot_trajectory::RobotTrajectoryPtr trajectory;
		if (!pair.second->plan(start, end, jmg, timeout, trajectory, path_constraints))
			break;

		sub_trajectories.push_back(trajectory);
		// continue from reached state
		start = end;
	}

	SolutionBase* solution = nullptr;
	if (sub_trajectories.size() != planner_.size())  { // error during sequential planning
		if (!storeFailures())
			return false;
		// push back a dummy solution to also show the target scene of the failed attempt
		sub_trajectories.push_back(robot_trajectory::RobotTrajectoryPtr());
		solution = storeSequential(sub_trajectories, intermediate_scenes);
		// mark solution as failure
		solution->setCost(std::numeric_limits<double>::infinity());
	} else {
		robot_trajectory::RobotTrajectoryConstPtr t = nullptr;
		if(sub_trajectories.size() >= 2)
			t = merge(sub_trajectories, intermediate_scenes, from.scene()->getCurrentState());
		else
			t = sub_trajectories[0];

		if (t) {
			connect(from, to, SubTrajectory(t));
			return true;
		}
		// merging failed, store sequentially
		solution = storeSequential(sub_trajectories, intermediate_scenes);
	}

	newSolution(from, to, *solution);
	return !solution->isFailure();
}

SolutionBase* Connect::storeSequential(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
                                       const std::vector<planning_scene::PlanningScenePtr>& intermediate_scenes)
{
	assert(sub_trajectories.size() + 1 == intermediate_scenes.size());
	auto scene_it = intermediate_scenes.begin();
	planning_scene::PlanningScenePtr start = *scene_it;

	SolutionSequence::container_type sub_solutions;
	for (const auto &sub : sub_trajectories) {
		planning_scene::PlanningScenePtr end = *++scene_it;

		auto inserted = subsolutions_.insert(subsolutions_.end(), SubTrajectory(sub));
		inserted->setCreator(pimpl_);
		// push back solution pointer
		sub_solutions.push_back(&*inserted);

		// provide meaningful start/end states
		states_.emplace_back(InterfaceState(start));
		subsolutions_.back().setStartState(states_.back());
		states_.emplace_back(InterfaceState(end));
		subsolutions_.back().setEndState(states_.back());

		start = end;
	}

	solutions_.emplace_back(SolutionSequence(std::move(sub_solutions)));
	return &solutions_.back();
}

robot_trajectory::RobotTrajectoryPtr Connect::merge(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
                                                    const std::vector<planning_scene::PlanningScenePtr>& intermediate_scenes,
                                                    const moveit::core::RobotState& state)
{
	auto jmg = merged_jmg_.get();
	robot_trajectory::RobotTrajectoryPtr trajectory = task_constructor::merge(sub_trajectories, state, jmg);
	// TODO: check merged trajectory for collisions
	return trajectory;
}

size_t Connect::numSolutions() const
{
	return solutions_.size() + Connecting::numSolutions();
}

void Connect::processSolutions(const Stage::SolutionProcessor& processor) const
{
	// TODO: This is not nice, but necessary to process simple SubTrajectory + SolutionSequence
	for (const auto& s : solutions_) {
		if (s.isFailure()) continue;
		if (!processor(s))
			break;
	}
	Connecting::processSolutions(processor);
}

void Connect::processFailures(const Stage::SolutionProcessor &processor) const
{
	// TODO: This is not nice, but necessary to process simple SubTrajectory + SolutionSequence
	for (const auto& s : solutions_) {
		if (!s.isFailure()) continue;
		if (!processor(s))
			break;
	}
	Connecting::processFailures(processor);
}

} } }
