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

/* Authors: Robert Haschke */

#pragma once
#include <list>

#include <actionlib/client/simple_action_client.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_execution_manager/trajectory_execution_manager.h>
#include <moveit/utils/moveit_error_code.h>

#include <moveit/task_constructor/storage.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <moveit_msgs/PlanningScene.h>

namespace moveit {
namespace task_constructor {
inline namespace execution {

class PlanExecution;
struct ExecutableTrajectory
{
	using EffectFn = std::function<void(PlanExecution*)>;

	std::string description;
	robot_trajectory::RobotTrajectoryConstPtr trajectory;
	std::vector<std::string> controller_names;
	// std::list doesn't invalidate iterators upon insertion/deletion
	std::list<EffectFn> start_effects;
	std::list<EffectFn> end_effects;
	bool stop = false;
};
// std::list doesn't invalidate iterators upon insertion/deletion
using ExecutableMotionPlan = std::list<ExecutableTrajectory>;

class PlanExecution
{
	planning_scene_monitor::PlanningSceneMonitorPtr psm_;
	trajectory_execution_manager::TrajectoryExecutionManagerPtr tem_;
	ExecutableMotionPlan components_;
	ExecutableMotionPlan::iterator next_;

	void call(const std::list<ExecutableTrajectory::EffectFn>& effects, const char* name);
	void onDone(const moveit_controller_manager::ExecutionStatus& status);
	void onSuccessfulComponent();

public:
	PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
	              const trajectory_execution_manager::TrajectoryExecutionManagerPtr& tem);

	void prepare(ExecutableMotionPlan components);
	moveit::core::MoveItErrorCode run();

	auto remaining() const { return std::distance<ExecutableMotionPlan::const_iterator>(next_, components_.end()); }

	const planning_scene_monitor::PlanningSceneMonitorPtr& getPlanningSceneMonitor() const { return psm_; }
	const trajectory_execution_manager::TrajectoryExecutionManagerPtr& getTrajectoryExecutionManager() const {
		return tem_;
	}
};

/// Execute trajectory using ExecuteTaskSolution provided as move_group capability
using ExecuteTaskSolutionSimpleActionClient =
    actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction>;
bool execute(const SolutionBase& s, ExecuteTaskSolutionSimpleActionClient* ac = nullptr, bool wait = true);

/// Construct a motion plan for execution with MoveIt's PlanExecution
ExecutableMotionPlan executableMotionPlan(const SolutionBase& s);

}  // namespace execution
}  // namespace task_constructor
}  // namespace moveit
