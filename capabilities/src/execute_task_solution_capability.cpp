/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Michael 'v4hn' Goerner */

#include "execute_task_solution_capability.h"

#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/utils/message_checks.h>
#include <moveit/moveit_cpp/moveit_cpp.h>

#include <boost/algorithm/string/join.hpp>

namespace {

// TODO: move to moveit::core::RobotModel
const moveit::core::JointModelGroup* findJointModelGroup(const moveit::core::RobotModel& model,
                                                         const std::vector<std::string>& joints) {
	std::set<std::string> joint_set(joints.begin(), joints.end());

	const std::vector<const moveit::core::JointModelGroup*>& jmgs = model.getJointModelGroups();

	for (const moveit::core::JointModelGroup* jmg : jmgs) {
		const std::vector<std::string>& jmg_joints = jmg->getJointModelNames();
		std::set<std::string> jmg_joint_set(jmg_joints.begin(), jmg_joints.end());

		// return group if sets agree on all active joints
		if (std::includes(jmg_joint_set.begin(), jmg_joint_set.end(), joint_set.begin(), joint_set.end())) {
			std::set<std::string> difference;
			std::set_difference(jmg_joint_set.begin(), jmg_joint_set.end(), joint_set.begin(), joint_set.end(),
			                    std::inserter(difference, difference.begin()));
			unsigned int acceptable = 0;
			for (const std::string& diff_joint : difference) {
				const moveit::core::JointModel* diff_jm = model.getJointModel(diff_joint);
				if (diff_jm->isPassive() || diff_jm->getMimic() || diff_jm->getType() == moveit::core::JointModel::FIXED)
					++acceptable;
			}
			if (difference.size() == acceptable)
				return jmg;
		}
	}

	return nullptr;
}
}  // namespace

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.execute_task_solution");

namespace move_group {

ExecuteTaskSolutionCapability::ExecuteTaskSolutionCapability() : MoveGroupCapability("ExecuteTaskSolution") {}

void ExecuteTaskSolutionCapability::initialize() {
	// configure the action server
	as_ = rclcpp_action::create_server<moveit_task_constructor_msgs::action::ExecuteTaskSolution>(
	    context_->moveit_cpp_->getNode(), "execute_task_solution",
	    [this](const rclcpp_action::GoalUUID& /*uuid*/,
	           const ExecuteTaskSolutionAction::Goal::ConstSharedPtr& /*goal*/) {
		    // Reject new goal if another goal is currently processed
		    if (last_goal_future_.valid() &&
		        last_goal_future_.wait_for(std::chrono::seconds::zero()) != std::future_status::ready) {
			    return rclcpp_action::GoalResponse::REJECT;
		    }
		    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	    },
	    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& goal_handle) {
		    return preemptCallback(goal_handle);
	    },
	    [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& goal_handle) {
		    last_goal_future_ =
		        std::async(std::launch::async, &ExecuteTaskSolutionCapability::goalCallback, this, goal_handle);
	    });
}

void ExecuteTaskSolutionCapability::goalCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& goal_handle) {
	auto result = std::make_shared<moveit_task_constructor_msgs::action::ExecuteTaskSolution::Result>();

	const auto& goal = goal_handle->get_goal();
	if (!context_->plan_execution_) {
		result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED;
		goal_handle->abort(result);
		return;
	}

	plan_execution::ExecutableMotionPlan plan;
	if (!constructMotionPlan(goal->solution, plan))
		result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
	else {
		RCLCPP_INFO(LOGGER, "Executing TaskSolution");
		result->error_code = context_->plan_execution_->executeAndMonitor(plan);
	}

	if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
		goal_handle->succeed(result);
	else if (result->error_code.val == moveit_msgs::msg::MoveItErrorCodes::PREEMPTED && goal_handle->is_canceling())
		goal_handle->canceled(result);
	else
		goal_handle->abort(result);
}

rclcpp_action::CancelResponse ExecuteTaskSolutionCapability::preemptCallback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ExecuteTaskSolutionAction>>& /*goal_handle*/) {
	if (context_->plan_execution_)
		context_->plan_execution_->stop();
	return rclcpp_action::CancelResponse::ACCEPT;
}

bool ExecuteTaskSolutionCapability::constructMotionPlan(const moveit_task_constructor_msgs::msg::Solution& solution,
                                                        plan_execution::ExecutableMotionPlan& plan) {
	moveit::core::RobotModelConstPtr model = context_->planning_scene_monitor_->getRobotModel();

	moveit::core::RobotState state(model);
	{
		planning_scene_monitor::LockedPlanningSceneRO scene(context_->planning_scene_monitor_);
		state = scene->getCurrentState();
	}

	plan.plan_components_.reserve(solution.sub_trajectory.size());
	for (size_t i = 0; i < solution.sub_trajectory.size(); ++i) {
		const moveit_task_constructor_msgs::msg::SubTrajectory& sub_traj = solution.sub_trajectory[i];

		plan.plan_components_.emplace_back();
		plan_execution::ExecutableTrajectory& exec_traj = plan.plan_components_.back();

		// define individual variable for use in closure below
		const std::string description = std::to_string(i + 1) + "/" + std::to_string(solution.sub_trajectory.size());
		exec_traj.description_ = description;

		const moveit::core::JointModelGroup* group = nullptr;
		{
			std::vector<std::string> joint_names(sub_traj.trajectory.joint_trajectory.joint_names);
			joint_names.insert(joint_names.end(), sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.begin(),
			                   sub_traj.trajectory.multi_dof_joint_trajectory.joint_names.end());
			if (!joint_names.empty()) {
				group = findJointModelGroup(*model, joint_names);
				if (!group) {
					RCLCPP_ERROR_STREAM(LOGGER, "Could not find JointModelGroup that actuates {"
					                                << boost::algorithm::join(joint_names, ", ") << "}");
					return false;
				}
				RCLCPP_DEBUG(LOGGER, "Using JointModelGroup '%s' for execution", group->getName().c_str());
			}
		}
		exec_traj.trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(model, group);
		exec_traj.trajectory_->setRobotTrajectoryMsg(state, sub_traj.trajectory);
		exec_traj.controller_names_ = sub_traj.execution_info.controller_names;

		/* TODO add action feedback and markers */
		exec_traj.effect_on_success_ = [this, sub_traj,
		                               description](const plan_execution::ExecutableMotionPlan* /*plan*/) {
			if (!moveit::core::isEmpty(sub_traj.scene_diff)) {
				RCLCPP_DEBUG_STREAM(LOGGER, "apply effect of " << description);
				return context_->planning_scene_monitor_->newPlanningSceneMessage(sub_traj.scene_diff);
			}
			return true;
		};

		if (!moveit::core::isEmpty(sub_traj.scene_diff.robot_state) &&
		    !moveit::core::robotStateMsgToRobotState(sub_traj.scene_diff.robot_state, state, true)) {
			RCLCPP_ERROR_STREAM(LOGGER, "invalid intermediate robot state in scene diff of SubTrajectory " << description);
			return false;
		}
	}

	return true;
}

}  // namespace move_group

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(move_group::ExecuteTaskSolutionCapability, move_group::MoveGroupCapability)
