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
#include <moveit_msgs/MoveItErrorCodes.h>

namespace moveit {
namespace task_constructor {

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

}  // namespace task_constructor
}  // namespace moveit
