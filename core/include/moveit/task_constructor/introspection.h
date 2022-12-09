/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Author: Robert Haschke
   Desc:   Introspection provides ROS interfaces to introspect Task instances
*/

#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor_msgs/msg/task_description.hpp>
#include <moveit_task_constructor_msgs/msg/task_statistics.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit_task_constructor_msgs/srv/get_solution.hpp>

#define DESCRIPTION_TOPIC "description"
#define STATISTICS_TOPIC "statistics"
#define SOLUTION_TOPIC "solution"
#define GET_SOLUTION_SERVICE "get_solution"

namespace moveit {
namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage);
MOVEIT_CLASS_FORWARD(SolutionBase);

class TaskPrivate;
class IntrospectionPrivate;

/** The Introspection class provides publishing of task state and solutions.
 *
 *  It is interlinked to its task.
 */
class Introspection
{
	IntrospectionPrivate* impl;

public:
	Introspection(const TaskPrivate* task);
	Introspection(const Introspection& other) = delete;
	~Introspection();

	/// fill task description message for publishing the task configuration
	moveit_task_constructor_msgs::msg::TaskDescription&
	fillTaskDescription(moveit_task_constructor_msgs::msg::TaskDescription& msg);
	/// publish detailed task description
	void publishTaskDescription();

	/// fill task state message for publishing the current task state
	moveit_task_constructor_msgs::msg::TaskStatistics&
	fillTaskStatistics(moveit_task_constructor_msgs::msg::TaskStatistics& msg);
	/// publish the current state of task
	void publishTaskState();

	/// indicate that this task was reset
	void reset();

	/// register the given solution, assigning a unique ID
	void registerSolution(const SolutionBase& s);

	/// publish the given solution
	void publishSolution(const SolutionBase& s);

	/// publish all top-level solutions of task
	void publishAllSolutions(bool wait = true);

	/// get solution
	bool getSolution(const moveit_task_constructor_msgs::srv::GetSolution::Request::SharedPtr& req,
	                 const moveit_task_constructor_msgs::srv::GetSolution::Response::SharedPtr& res);

	/// retrieve id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage* const s) const;

	/// retrieve or set id of given solution
	uint32_t solutionId(const moveit::task_constructor::SolutionBase& s);

private:
	void fillStageStatistics(const Stage& stage, moveit_task_constructor_msgs::msg::StageStatistics& s);
	void fillSolution(moveit_task_constructor_msgs::msg::Solution& msg, const SolutionBase& s);
	/// retrieve or set id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage* const s);
	/// retrieve solution with given id
	const SolutionBase* solutionFromId(uint id) const;
};
}  // namespace task_constructor
}  // namespace moveit
