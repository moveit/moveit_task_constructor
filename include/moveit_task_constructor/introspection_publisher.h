#pragma once

#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>
#include <moveit_task_constructor/Task.h>

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SolutionBase)

class IntrospectionPublisher {
	ros::Publisher pub_;

public:
	IntrospectionPublisher(const std::string &topic = "tasks");
	static IntrospectionPublisher &instance();

	// publish the current state of given task
	void publish(const Task &t);
};

} }
