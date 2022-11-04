/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021 PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Boston Cleek
   Desc:   Abstact class for stages using a simple action client.
*/

#include <moveit/task_constructor/stages/action_base.h>

namespace moveit {
namespace task_constructor {
namespace stages {

constexpr char LOGNAME[] = "action_base";

ActionBase::ActionBase(const std::string& action_name, bool spin_thread, double goal_timeout, double server_timeout)
  : action_name_(action_name), server_timeout_(server_timeout), goal_timeout_(goal_timeout) {
	clientPtr_ = std::make_unique<actionlib::SimpleActionClient<grasping_msgs::GraspPlanningAction>>(nh_, action_name_,
	                                                                                                 spin_thread);

	// Negative timeouts are set to zero
	server_timeout_ = server_timeout_ < std::numeric_limits<double>::epsilon() ? 0.0 : server_timeout_;
	goal_timeout_ = goal_timeout_ < std::numeric_limits<double>::epsilon() ? 0.0 : goal_timeout_;

	ROS_DEBUG_STREAM_NAMED(LOGNAME, "Waiting for connection to grasp generation action server...");
	clientPtr_->waitForServer(ros::Duration(server_timeout_));
	ROS_DEBUG_STREAM_NAMED(LOGNAME, "Connected to server");
}

ActionBase::ActionBase(const std::string& action_name, bool spin_thread)
  : ActionBase::ActionBase(action_name, spin_thread, 0.0, 0.0) {}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
