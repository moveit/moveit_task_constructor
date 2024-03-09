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

#include <moveit_msgs/msg/constraints.hpp>

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotState);
}
}  // namespace moveit

namespace moveit {
namespace task_constructor {
namespace stages {

/** Connect arbitrary InterfaceStates by motion planning
 *
 * The states may differ in various planning groups.
 * To connect both states, the planners provided for individual sub groups are applied in the
 * specified order. Each planner only plan for joints within the corresponding planning group.
 * Finally, an attempt is made to merge the sub trajectories of individual planning results.
 * If this fails, the sequential planning result is returned.
 */
class Connect : public Connecting
{
protected:
	bool compatible(const InterfaceState& from_state, const InterfaceState& to_state) const override;

public:
	enum MergeMode
	{
		SEQUENTIAL = 0,
		WAYPOINTS = 1
	};

	struct PlannerIdTrajectoryPair
	{
		std::string planner_id;
		robot_trajectory::RobotTrajectoryConstPtr trajectory;
	};

	using GroupPlannerVector = std::vector<std::pair<std::string, solvers::PlannerInterfacePtr>>;
	Connect(const std::string& name = "connect", const GroupPlannerVector& planners = {});

	void setMaxDistance(double max_distance) { setProperty("max_distance", max_distance); }
	void setPathConstraints(moveit_msgs::msg::Constraints path_constraints) {
		setProperty("path_constraints", std::move(path_constraints));
	}

	void reset() override;
	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	void compute(const InterfaceState& from, const InterfaceState& to) override;

protected:
	SolutionSequencePtr makeSequential(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
	                                   const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
	                                   const InterfaceState& from, const InterfaceState& to);
	SubTrajectoryPtr merge(const std::vector<PlannerIdTrajectoryPair>& sub_trajectories,
	                       const std::vector<planning_scene::PlanningSceneConstPtr>& intermediate_scenes,
	                       const moveit::core::RobotState& state);

protected:
	GroupPlannerVector planner_;
	moveit::core::JointModelGroupPtr merged_jmg_;
	std::list<SubTrajectory> subsolutions_;
	std::list<InterfaceState> states_;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
