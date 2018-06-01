/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner, Robert Haschke
   Desc:    User-level access to the whole Task constructor infrastructure
*/

#pragma once

#include "container.h"

#include <moveit/task_constructor/introspection.h>
#include <moveit_task_constructor_msgs/Solution.h>

#include <moveit/macros/class_forward.h>

namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotModel)
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace robot_model_loader {
	MOVEIT_CLASS_FORWARD(RobotModelLoader)
}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(ContainerBase)
MOVEIT_CLASS_FORWARD(Task)

/** A Task is the root of a tree of stages.
 *
 * Actually a tasks wraps a single container (by default a SerialContainer),
 * which serves as the root of all stages.
 */
class Task : protected WrapperBase {
public:
	// +1 TODO: move into MoveIt! core
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");
	Task(const std::string& id = "",
	     ContainerBase::pointer &&container = std::make_unique<SerialContainer>("task pipeline"));
	Task(Task &&other);
	Task& operator=(Task&& other);
	~Task();

	std::string id() const;
	const moveit::core::RobotModelConstPtr getRobotModel() const { return robot_model_; }
	/// setting the robot model also resets the task
	void setRobotModel(const moveit::core::RobotModelConstPtr& robot_model);
	/// load robot model from given parameter
	void loadRobotModel(const std::string& robot_description = "robot_description");

	// TODO: use Stage::insert as well?
	void add(Stage::pointer &&stage);
	void clear() override;

	/// enable introspection publishing for use with rviz
	void enableIntrospection(bool enable = true);
	Introspection &introspection();

	typedef std::function<void(const Task &t)> TaskCallback;
	typedef std::list<TaskCallback> TaskCallbackList;
	/// add function to be called after each top-level iteration
	TaskCallbackList::const_iterator addTaskCallback(TaskCallback &&cb);
	/// remove function callback
	void erase(TaskCallbackList::const_iterator which);

	/// reset all stages
	void reset() override;
	/// initialize all stages with given scene
	void init();

	/// reset, init scene (if not yet done), and init all stages, then start planning
	bool plan(size_t max_solutions = 0);
	/// interrupt current planning (or execution)
	void preempt();

	/// print current task state (number of found solutions and propagated states) to std::cout
	void printState(std::ostream &os = std::cout) const;

	size_t numSolutions() const { return solutions().size(); }
	const ordered<SolutionBaseConstPtr>& solutions() const { return stages()->solutions(); }
	const std::list<SolutionBaseConstPtr>& failures() const { return stages()->failures(); }

	/// publish all top-level solutions
	void publishAllSolutions(bool wait = true);

	// +1 TODO: convenient access to arbitrary stage by name. traverse hierarchy using / separator?
	/// access stage tree
	ContainerBase *stages();
	const ContainerBase *stages() const;

	/// properties access
	PropertyMap& properties();
	const PropertyMap& properties() const {
		return const_cast<Task*>(this)->properties();
	}
	void setProperty(const std::string& name, const boost::any& value);
	/// overload: const char* values are stored as std::string
	inline void setProperty(const std::string& name, const char* value) {
		setProperty(name, std::string(value));
	}

protected:
	bool canCompute() const override;
	void compute() override;
	void onNewSolution(const SolutionBase &s) override;

private:
	std::string id_;
	robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
	moveit::core::RobotModelConstPtr robot_model_;
	bool preempt_requested_;

	// introspection and monitoring
	std::unique_ptr<Introspection> introspection_;
	std::list<Task::TaskCallback> task_cbs_; // functions to monitor task's planning progress
};

inline std::ostream& operator<<(std::ostream& os, const Task& task) {
	task.printState(os);
	return os;
}

} }
