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
#include <moveit/planning_scene/planning_scene.h>

namespace moveit { namespace task_constructor { namespace stages {

Connect::Connect(std::string name, const GroupPlannerVector& planners)
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

	for (const GroupPlannerVector::value_type& pair : planner_) {
		if (!robot_model->hasJointModelGroup(pair.first))
			errors.push_back(*this, "invalid group: " + pair.first);
		else if (!pair.second)
			errors.push_back(*this, "invalid planner for group: " + pair.first);
		else
			pair.second->init(robot_model);
	}

	if (errors)
		throw errors;
}

bool Connect::compute(const InterfaceState &from, const InterfaceState &to) {
	const auto& props = properties();
	double timeout = props.get<double>("timeout");
	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");

	SolutionSequence::container_type subsolutions;
	planning_scene::PlanningScenePtr start = from.scene()->diff();
	const moveit::core::RobotState& goal_state = to.scene()->getCurrentState();

	for (const GroupPlannerVector::value_type& pair : planner_) {
		// set intermediate goal state
		planning_scene::PlanningScenePtr end = start->diff();
		const moveit::core::JointModelGroup *jmg = goal_state.getJointModelGroup(pair.first);
		std::vector<double> positions;
		goal_state.copyJointGroupPositions(jmg, positions);
		end->getCurrentStateNonConst().setJointGroupPositions(jmg, positions);

		robot_trajectory::RobotTrajectoryPtr trajectory;
		bool success = pair.second->plan(start, to.scene(), jmg, timeout, trajectory, path_constraints);

		// store solution
		auto inserted = subsolutions_.insert(subsolutions_.end(), SubTrajectory(trajectory));
		inserted->setCreator(pimpl_);
		// push back solution pointer
		subsolutions.push_back(&*inserted);

		// provide meaningful start/end states
		states_.emplace_back(InterfaceState(start));
		subsolutions_.back().setStartState(states_.back());
		states_.emplace_back(InterfaceState(end));
		subsolutions_.back().setEndState(states_.back());
		if (!success) return false;

		// continue from reached state
		start = end;
	}

	solutions_.emplace_back(SolutionSequence(std::move(subsolutions), 0.0));
	newSolution(from, to, solutions_.back());
	return true;
}

void Connect::processSolutions(const Stage::SolutionProcessor& processor) const
{

}

} } }
