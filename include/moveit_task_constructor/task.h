// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include "container.h"

#include <moveit/macros/class_forward.h>

#include <ros/ros.h>

#include <vector>

namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(SubTask)
MOVEIT_CLASS_FORWARD(ContainerBase)
MOVEIT_CLASS_FORWARD(Task)

class TaskPrivate;
class Task : protected SerialContainer {
	PRIVATE_CLASS(Task)
public:
	typedef std::function<bool(const std::vector<SubTrajectory*>&)> SolutionCallback;

	Task(const std::string &name = std::string());

	void add(SubTask::pointer &&stage);
	using SerialContainer::clear;

	bool plan();
	void printState();

	const moveit::core::RobotState &getCurrentRobotState() const;
	bool processSolutions(const SolutionCallback &processor);
	bool processSolutions(const SolutionCallback &processor) const;
};

} }
