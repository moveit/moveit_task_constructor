// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "container.h"

#include <moveit/macros/class_forward.h>

namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotModel)
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(ContainerBase)
MOVEIT_CLASS_FORWARD(Task)

class TaskPrivate;
class Task : protected SerialContainer {
	PRIVATE_CLASS(Task)
public:
	Task(const std::string &name = std::string());
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");

	void add(Stage::pointer &&stage);
	using SerialContainer::clear;

	bool plan();
	void printState();

	const moveit::core::RobotState &getCurrentRobotState() const;

#if 0
	bool processSolutions(const SolutionCallback &processor);
	bool processSolutions(const SolutionCallback &processor) const;
#endif
};

} }
