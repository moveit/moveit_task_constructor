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

/* Authors: Robert Haschke, Michael Goerner
   Desc:    Connect arbitrary states by motion planning
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/solvers/planner_interface.h>

#include <moveit_msgs/Constraints.h>

namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(RobotState)
} }

namespace moveit { namespace task_constructor { namespace stages {

class Connect : public Connecting {
protected:
	bool compatible(const InterfaceState &from_state, const InterfaceState &to_state) const override;

public:
	typedef std::vector<std::pair<std::string, solvers::PlannerInterfacePtr>> GroupPlannerVector;
	Connect(const std::string& name, const GroupPlannerVector& planners);

	void setTimeout(const ros::Duration& timeout){
		setProperty("timeout", timeout.toSec());
	}

	void setPathConstraints(moveit_msgs::Constraints path_constraints){
		setProperty("path_constraints", std::move(path_constraints));
	}

	void reset() override;
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool compute(const InterfaceState &from, const InterfaceState &to) override;

	size_t numSolutions() const override;
	void processSolutions(const SolutionProcessor &processor) const override;
	void processFailures(const SolutionProcessor &processor) const override;

protected:
	SolutionBase* storeSequential(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
	                              const std::vector<planning_scene::PlanningScenePtr>& intermediate_scenes);
	robot_trajectory::RobotTrajectoryConstPtr merge(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
	                                                const std::vector<planning_scene::PlanningScenePtr>& intermediate_scenes,
	                                                const moveit::core::RobotState& state);

protected:
	GroupPlannerVector planner_;
	moveit::core::JointModelGroupPtr merged_jmg_;
	// TODO: ComputeBase should handle any SolutionBase -> shared_ptr
	std::list<SubTrajectory> subsolutions_;
	std::list<SolutionSequence> solutions_;
	std::list<InterfaceState> states_;
};

} } }
