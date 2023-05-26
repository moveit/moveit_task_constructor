/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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
 *   * Neither the name of PickNik Inc. nor the names of its
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

/* Authors: Sebastian Jahr, Robert Haschke
   Desc:    Meta planner, running multiple planners in parallel
*/

#pragma once

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/planning_pipeline_interfaces/plan_responses_container.hpp>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/planning_pipeline_interfaces/solution_selection_functions.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <vector>

namespace moveit {
namespace task_constructor {
namespace solvers {

MOVEIT_CLASS_FORWARD(AlternativesPlanner);

/** A meta planner that runs multiple alternative planners in parallel and returns the best solution.
 *
 * This is useful to try different planning strategies of increasing complexity,
 * e.g. Cartesian or joint-space interpolation and OMPL, and choose the most suitable solution out of the ones produced.
 */
class AlternativesPlanner : public PlannerInterface, public std::vector<solvers::PlannerInterfacePtr>
{
public:
	using PlannerList = std::vector<solvers::PlannerInterfacePtr>;
	using PlannerList::PlannerList;  // inherit all std::vector constructors

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	bool plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
	          const moveit::core::JointModelGroup* jmg, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
	          const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

	bool plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
	          const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg,
	          double timeout, robot_trajectory::RobotTrajectoryPtr& result,
	          const moveit_msgs::msg::Constraints& path_constraints = moveit_msgs::msg::Constraints()) override;

	/** \brief Set solution selection function for parallel planning
	 * \param [in] solution_selection_function New solution selection that will be used
	 */
	void setSolutionSelectionFunction(
	    const moveit::planning_pipeline_interfaces::SolutionSelectionFunction& solution_selection_function) {
		solution_selection_function_ = solution_selection_function;
	};

protected:
	moveit::planning_pipeline_interfaces::SolutionSelectionFunction solution_selection_function_ =
	    &moveit::planning_pipeline_interfaces::getShortestSolution;
};
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
