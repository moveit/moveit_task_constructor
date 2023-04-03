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

#include <moveit/task_constructor/solvers/alternatives_planner.h>
#include <moveit/robot_state/conversions.h>
#include <chrono>
#include <thread>

namespace {
const auto LOGGER = rclcpp::get_logger("AlternativesPlanner");
using clock = std::chrono::high_resolution_clock;
}  // namespace
namespace moveit {
namespace task_constructor {
namespace solvers {

void AlternativesPlanner::init(const core::RobotModelConstPtr& robot_model) {
	for (const auto& planner : *this)
		planner->init(robot_model);
}

bool AlternativesPlanner::plan(const planning_scene::PlanningSceneConstPtr& from,
                               const planning_scene::PlanningSceneConstPtr& to,
                               const moveit::core::JointModelGroup* jmg, double timeout,
                               robot_trajectory::RobotTrajectoryPtr& result,
                               const moveit_msgs::msg::Constraints& path_constraints) {
	moveit::planning_pipeline_interfaces::PlanResponsesContainer plan_responses_container{ this->size() };
	std::vector<std::thread> planning_threads;
	planning_threads.reserve(this->size());

	// Print a warning if more parallel planning problems than available concurrent threads are defined. If
	// std::thread::hardware_concurrency() is not defined, the command returns 0 so the check does not work
	auto const hardware_concurrency = std::thread::hardware_concurrency();
	if (planning_threads.size() > hardware_concurrency && hardware_concurrency != 0) {
		RCLCPP_WARN(LOGGER,
		            "More parallel planning problems defined ('%ld') than possible to solve concurrently with the "
		            "hardware ('%d')",
		            planning_threads.size(), hardware_concurrency);
	}

	// Start one planning thread for each available planner
	for (const auto& planner : *this) {
		auto planning_thread = std::thread([&]() {
			// Create trajectory to store planning result in
			robot_trajectory::RobotTrajectoryPtr trajectory;

			// Create motion plan response for future evaluation
			auto plan_solution = ::planning_interface::MotionPlanResponse();
			moveit::core::robotStateToRobotStateMsg(from->getCurrentState(), plan_solution.start_state);

			// Run planner
			auto const t1 = clock::now();
			bool success = planner->plan(from, to, jmg, timeout, result, path_constraints);
			plan_solution.planning_time = std::chrono::duration<double>(clock::now() - t1).count();

			if (success) {
				plan_solution.error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
				plan_solution.trajectory = trajectory;
			} else {
				plan_solution.error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
			}
			plan_responses_container.pushBack(plan_solution);
		});

		planning_threads.push_back(std::move(planning_thread));
	}

	// Wait for threads to finish
	for (auto& planning_thread : planning_threads) {
		if (planning_thread.joinable()) {
			planning_thread.join();
		}
	}

	// Select solution
	if (!solution_selection_function_) {
		RCLCPP_ERROR(LOGGER, "No solution selection function defined! Cannot choose the best solution so this planner "
		                     "returns failure.");
		return false;
	}

	std::vector<::planning_interface::MotionPlanResponse> solutions;
	solutions.reserve(1);
	solutions.push_back(solution_selection_function_(plan_responses_container.getSolutions()));

	if (solutions.empty()) {
		return false;
	}
	result = solutions.at(1).trajectory;
	return bool(solutions.at(1));
}

bool AlternativesPlanner::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                               const Eigen::Isometry3d& offset, const Eigen::Isometry3d& target,
                               const moveit::core::JointModelGroup* jmg, double timeout,
                               robot_trajectory::RobotTrajectoryPtr& result,
                               const moveit_msgs::msg::Constraints& path_constraints) {
	moveit::planning_pipeline_interfaces::PlanResponsesContainer plan_responses_container{ this->size() };
	std::vector<std::thread> planning_threads;
	planning_threads.reserve(this->size());

	// Print a warning if more parallel planning problems than available concurrent threads are defined. If
	// std::thread::hardware_concurrency() is not defined, the command returns 0 so the check does not work
	auto const hardware_concurrency = std::thread::hardware_concurrency();
	if (planning_threads.size() > hardware_concurrency && hardware_concurrency != 0) {
		RCLCPP_WARN(LOGGER,
		            "More parallel planning problems defined ('%ld') than possible to solve concurrently with the "
		            "hardware ('%d')",
		            planning_threads.size(), hardware_concurrency);
	}

	// Start one planning thread for each available planner
	for (const auto& planner : *this) {
		auto planning_thread = std::thread([&]() {
			// Create trajectory to store planning result in
			robot_trajectory::RobotTrajectoryPtr trajectory;

			// Create motion plan response for future evaluation
			auto plan_solution = ::planning_interface::MotionPlanResponse();
			moveit::core::robotStateToRobotStateMsg(from->getCurrentState(), plan_solution.start_state);

			// Run planner
			auto const t1 = clock::now();
			bool success = planner->plan(from, link, offset, target, jmg, timeout, trajectory, path_constraints);
			plan_solution.planning_time = std::chrono::duration<double>(clock::now() - t1).count();

			if (success) {
				plan_solution.error_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
				plan_solution.trajectory = trajectory;
			} else {
				plan_solution.error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
			}
			plan_responses_container.pushBack(plan_solution);
		});

		planning_threads.push_back(std::move(planning_thread));
	}

	// Wait for threads to finish
	for (auto& planning_thread : planning_threads) {
		if (planning_thread.joinable()) {
			planning_thread.join();
		}
	}

	// Select solution
	if (!solution_selection_function_) {
		RCLCPP_ERROR(LOGGER, "No solution selection function defined! Cannot choose the best solution so this planner "
		                     "returns failure.");
		return false;
	}

	std::vector<::planning_interface::MotionPlanResponse> solutions;
	solutions.reserve(1);
	solutions.push_back(solution_selection_function_(plan_responses_container.getSolutions()));

	if (solutions.empty()) {
		return false;
	}
	result = solutions.at(1).trajectory;
	return bool(solutions.at(1));
}
}  // namespace solvers
}  // namespace task_constructor
}  // namespace moveit
