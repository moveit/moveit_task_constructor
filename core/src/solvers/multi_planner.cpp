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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    generate and validate a straight-line Cartesian path
*/

#include <moveit/task_constructor/solvers/multi_planner.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <chrono>

namespace moveit {
namespace task_constructor {
namespace solvers {

void MultiPlanner::init(const core::RobotModelConstPtr& robot_model) {
	for (const auto& p : *this)
		p->init(robot_model);
}

PlannerInterface::Result MultiPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                            const planning_scene::PlanningSceneConstPtr& to,
                                            const moveit::core::JointModelGroup* jmg, double timeout,
                                            robot_trajectory::RobotTrajectoryPtr& result,
                                            const moveit_msgs::msg::Constraints& path_constraints) {
	double remaining_time = std::min(timeout, properties().get<double>("timeout"));
	auto start_time = std::chrono::steady_clock::now();

	std::string comment = "No planner specified";
	for (const auto& p : *this) {
		if (remaining_time < 0)
			return { false, "timeout" };
		if (result)
			result->clear();
		auto r = p->plan(from, to, jmg, remaining_time, result, path_constraints);
		if (r)
			return r;
		else
			comment = r.message;

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;
	}
	return { false, comment };
}

PlannerInterface::Result MultiPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                                            const moveit::core::LinkModel& link, const Eigen::Isometry3d& offset,
                                            const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg,
                                            double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                                            const moveit_msgs::msg::Constraints& path_constraints) {
	double remaining_time = std::min(timeout, properties().get<double>("timeout"));
	auto start_time = std::chrono::steady_clock::now();

	std::string comment = "No planner specified";
	for (const auto& p : *this) {
		if (remaining_time < 0)
			return { false, "timeout" };
		if (result)
			result->clear();
		auto r = p->plan(from, link, offset, target, jmg, remaining_time, result, path_constraints);
		if (r)
			return r;
		else
			comment = r.message;

		auto now = std::chrono::steady_clock::now();
		remaining_time -= std::chrono::duration<double>(now - start_time).count();
		start_time = now;
	}
	return { false, comment };
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
