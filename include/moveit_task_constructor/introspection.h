#pragma once

#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <moveit_task_constructor/TaskDescription.h>
#include <moveit_task_constructor/TaskStatistics.h>
#include <moveit_task_constructor/Solution.h>
#include <moveit_task_constructor/GetSolution.h>

#define DESCRIPTION_TOPIC "description"
#define STATISTICS_TOPIC  "statistics"
#define SOLUTION_TOPIC    "solution"

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SolutionBase)

class Introspection {
	// publish task detailed description and current state
	ros::Publisher task_description_publisher_;
	ros::Publisher task_statistics_publisher_;
	// publish new solutions
	ros::Publisher solution_publisher_;
	// services to provide an individual Solution
	ros::ServiceServer get_solution_service_;

public:
	Introspection();
	Introspection(const Introspection &other) = delete;
	static Introspection &instance();

	// publish detailed task description
	void publishTaskDescription(const Task &t);
	void publishTaskDescription(const ::moveit_task_constructor::TaskDescription &msg) {
		task_description_publisher_.publish(msg);
	}

	// publish the current state of given task
	void publishTaskState(const Task &t);
	void publishTaskState(const ::moveit_task_constructor::TaskStatistics &msg) {
		task_statistics_publisher_.publish(msg);
	}

	void operator()(const Task &t) { publishTaskState(t); }

	// publish the given solution
	void publishSolution(const SolutionBase &s);
	void publishSolution(const ::moveit_task_constructor::Solution &msg) {
		solution_publisher_.publish(msg);
	}
	void operator()(const SolutionBase &s) { publishSolution(s); }

	// get solution
	bool getSolution(moveit_task_constructor::GetSolution::Request  &req,
	                 moveit_task_constructor::GetSolution::Response &res);
};

void publishAllPlans(const Task &task, bool wait = true);

} }
