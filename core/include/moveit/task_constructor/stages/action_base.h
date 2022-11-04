/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2020 PickNik Inc.
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

#pragma once

#include <memory>
#include <string>
#include <limits>

#include <actionlib/client/simple_action_client.h>
#include <grasping_msgs/GraspPlanningAction.h>

namespace moveit {
namespace task_constructor {
namespace stages {

/** @brief Interface allowing stages to use a simple action client */
class ActionBase
{
public:
	/**
	 * @brief Constructor
	 * @param action_name - action namespace
	 * @param spin_thread - spins a thread to service this action's subscriptions
	 * @param goal_timeout - goal to completed time out (0 is considered infinite timeout)
	 * @param server_timeout - connection to server time out (0 is considered infinite timeout)
	 * @details Initialize the action client and time out parameters
	 */
	ActionBase(const std::string& action_name, bool spin_thread, double goal_timeout, double server_timeout);

	/**
	 * @brief Constructor
	 * @param action_name - action namespace
	 * @param spin_thread - spins a thread to service this action's subscriptions
	 * @details Initialize the action client and time out parameters to infinity
	 */
	ActionBase(const std::string& action_name, bool spin_thread);

	/* @brief Destructor */
	virtual ~ActionBase() = default;

	/* @brief Called when goal becomes active */
	virtual void activeCallback() = 0;

	/**
	 * @brief Called every time feedback is received for the goal
	 * @param feedback - pointer to the feedback message
	 */
	virtual void feedbackCallback(const grasping_msgs::GraspPlanningFeedbackConstPtr& feedback) = 0;

	/**
	 * @brief Called once when the goal completes
	 * @param state - state info for goal
	 * @param result - pointer to result message
	 */
	virtual void doneCallback(const actionlib::SimpleClientGoalState& state,
	                          const grasping_msgs::GraspPlanningResultConstPtr& result) = 0;

protected:
	ros::NodeHandle nh_;
	std::string action_name_;  // action name space
	std::unique_ptr<actionlib::SimpleActionClient<grasping_msgs::GraspPlanningAction>> clientPtr_;  // action client
	double server_timeout_, goal_timeout_;  // connection and goal completed time out
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
