/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
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

/* Authors: Joe Schornak
   Desc:    Define a struct to hold options used to configure trajectory execution
*/

#pragma once

#include <moveit_task_constructor_msgs/msg/trajectory_execution_info.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

namespace {
using TrajectoryExecutionInfoMsg = moveit_task_constructor_msgs::msg::TrajectoryExecutionInfo;
}

namespace moveit {
namespace task_constructor {
struct TrajectoryExecutionInfo
{
	TrajectoryExecutionInfo() {}

	TrajectoryExecutionInfo(const TrajectoryExecutionInfoMsg& msg) { controller_names = msg.controller_names; }

	TrajectoryExecutionInfoMsg toMsg() const {
		return moveit_task_constructor_msgs::build<TrajectoryExecutionInfoMsg>().controller_names(controller_names);
	}

	std::vector<std::string> controller_names;
};

}  // namespace task_constructor
}  // namespace moveit
