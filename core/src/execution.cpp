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

#include <moveit/task_constructor/execution.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/message_checks.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit/task_constructor/stage.h>

using namespace moveit_controller_manager;

namespace moveit {
namespace task_constructor {
inline namespace execution {

static const std::string LOGGER("execution");

PlanExecution::PlanExecution(const planning_scene_monitor::PlanningSceneMonitorPtr& psm,
                             const trajectory_execution_manager::TrajectoryExecutionManagerPtr& tem)
  : psm_(psm), tem_(tem) {}

void PlanExecution::prepare(ExecutableMotionPlan components) {
	components_ = std::move(components);
	next_ = components_.begin();
}

moveit::core::MoveItErrorCode PlanExecution::run() {
	if (next_ == components_.end())  // empty / already done
		return moveit_msgs::MoveItErrorCodes::SUCCESS;

	// push components to TEM
	for (auto it = next_, end = components_.end(); it != end; ++it) {
		moveit_msgs::RobotTrajectory msg;
		if (it->trajectory && !it->trajectory->empty() && (it->trajectory->getDuration() > 0.0))
			it->trajectory->getRobotTrajectoryMsg(msg);

		if (!tem_->push(msg, it->controller_names)) {
			ROS_ERROR_STREAM_NAMED(LOGGER, "Trajectory initialization failed");
			tem_->clear();
			next_ = end;
			return moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
		} else if (it->stop)
			break;
	}

	call(next_->start_effects, "start");
	tem_->execute([this](const ExecutionStatus& status) { onDone(status); },
	              [this](std::size_t) { onSuccessfulComponent(); });

	auto result = tem_->waitForExecution();
	ROS_DEBUG_STREAM_NAMED(LOGGER, remaining() << " segments remaining");
	switch (result) {
		case ExecutionStatus::SUCCEEDED:
			return moveit_msgs::MoveItErrorCodes::SUCCESS;
		case ExecutionStatus::PREEMPTED:
			return moveit_msgs::MoveItErrorCodes::PREEMPTED;
		case ExecutionStatus::TIMED_OUT:
			return moveit_msgs::MoveItErrorCodes::TIMED_OUT;
		case ExecutionStatus::ABORTED:
			return moveit_msgs::MoveItErrorCodes::ABORT;
		default:
			return moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
	}
}

void PlanExecution::call(const std::vector<ExecutableTrajectory::EffectFn>& effects, const char* name) {
	ROS_DEBUG_STREAM_NAMED(LOGGER + ".apply", effects.size() << " " << name << " effects of: " << next_->description);
	for (const auto& f : effects)
		f(this);
}

void PlanExecution::onDone(const ExecutionStatus& /*status*/) {}
void PlanExecution::onSuccessfulComponent() {
	call(next_->end_effects, "end");
	bool stop = next_->stop;  // stop on user request
	if (++next_ == components_.end())
		stop = true;  // stop on end of components
	if (!stop)
		call(next_->start_effects, "start");
}

bool execute(const SolutionBase& s, ExecuteTaskSolutionSimpleActionClient* ac, bool wait) {
	std::unique_ptr<ExecuteTaskSolutionSimpleActionClient> fallback;
	if (!ac) {
		fallback = std::make_unique<ExecuteTaskSolutionSimpleActionClient>("execute_task_solution");
		ac = fallback.get();
	}
	ac->waitForServer();

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal goal;
	s.fillMessage(goal.solution);
	s.start()->scene()->getPlanningSceneMsg(goal.solution.start_scene);

	ac->sendGoal(goal);
	if (wait) {
		ac->waitForResult();
		return ac->getResult()->error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS;
	}
	return true;
}

ExecutableMotionPlan executableMotionPlan(const SolutionBase& s) {
	ExecutableMotionPlan plan;
	auto f = [&](const SubTrajectory& sub) {
		ExecutableTrajectory t;
		t.description = sub.creator()->name();
		t.trajectory = sub.trajectory();
		plan.push_back(std::move(t));
		auto& c = plan.back();

		// apply planning scene changes (assuming only no-trajectory solution have those)
		if (!sub.trajectory()) {
			c.end_effects.push_back([scene = sub.end()->scene()](PlanExecution* plan) {
				if (!plan->getPlanningSceneMonitor())
					return;
				moveit_msgs::PlanningScene diff;
				scene->getPlanningSceneDiffMsg(diff);
				plan->getPlanningSceneMonitor()->newPlanningSceneMessage(diff);
			});
		}
	};
	s.visitSubTrajectories(f);
	return plan;
}

}  // namespace execution
}  // namespace task_constructor
}  // namespace moveit
