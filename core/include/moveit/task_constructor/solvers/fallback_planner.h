/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, Bielefeld University
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

/* Authors: Robert Haschke, Sebastian Jahr
   Desc:    Meta planner, running multiple MTC planners in sequence
*/

#pragma once

#include <moveit/task_constructor/solvers/planner_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <vector>

namespace moveit {
namespace task_constructor {
namespace solvers {

/** \brief A stopping criterion which is evaluated after running a planner.
 * \param [in] result Most recent result of the last planner invoked
 * \param [in] path_constraints Set of path constraints passed to the solver
 * \return  If it returns true, the corresponding trajectory is considered the solution, otherwise false is returned and
 * the next fallback planner is called.
 */
typedef std::function<bool(const robot_trajectory::RobotTrajectoryPtr& result,
                           const moveit_msgs::msg::Constraints& path_constraints)>
    StoppingCriterionFunction;

MOVEIT_CLASS_FORWARD(FallbackPlanner);

/** A meta planner that runs multiple alternative planners in sequence and returns the first found solution.
 *
 * This is useful to sequence different planning strategies of increasing complexity,
 * e.g. Cartesian or joint-space interpolation first, then OMPL, ...
 * This is (slightly) different from the Fallbacks container, as the FallbackPlanner directly applies its planners to
 * each individual planning job. In contrast, the Fallbacks container first runs the active child to exhaustion before
 * switching to the next child, which possibly applies a different planning strategy.
 */
class FallbackPlanner : public PlannerInterface, public std::vector<solvers::PlannerInterfacePtr>
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

	/** \brief Set stopping criterion function for parallel planning
	 * \param [in] stopping_criterion_callback New stopping criterion function that will be used
	 */
	void setStoppingCriterionFunction(const StoppingCriterionFunction& stopping_criterion_callback) {
		stopping_criterion_function_ = stopping_criterion_callback;
	}

protected:
	StoppingCriterionFunction stopping_criterion_function_ =
	    [](const robot_trajectory::RobotTrajectoryPtr& result,
	       const moveit_msgs::msg::Constraints& /*path_constraints*/) {
		    if (result != nullptr) {
			    return true;
		    }
		    return false;
	    };
};
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
