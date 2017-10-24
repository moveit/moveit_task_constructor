#pragma once

#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <moveit_task_constructor/Task.h>
#include <moveit_task_constructor/Solution.h>
#include <moveit_task_constructor/GetInterfaceState.h>
#include <moveit_task_constructor/GetSolution.h>

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SolutionBase)

#define DEFAULT_TASK_MONITOR_TOPIC "task_monitoring"
#define DEFAULT_TASK_SOLUTION_TOPIC "task_solutions"

class Introspection {
	// publish Task state and (new) Solutions
	ros::Publisher task_publisher_;
	ros::Publisher solution_publisher_;
	// services to provide InterfaceState and Solution
	ros::ServiceServer get_interface_state_service_;
	ros::ServiceServer get_solution_service_;

public:
	Introspection(const std::string &task_topic = DEFAULT_TASK_MONITOR_TOPIC,
	              const std::string &solution_topic = DEFAULT_TASK_SOLUTION_TOPIC);
	Introspection(const Introspection &other) = delete;
	static Introspection &instance();

	// publish the current state of given task
	void publishTask(const Task &t);
	void publishTask(const ::moveit_task_constructor::Task &msg) {
		task_publisher_.publish(msg);
	}

	void operator()(const Task &t) { publishTask(t); }

	// publish the given solution
	void publishSolution(const SolutionBase &s);
	void publishSolution(const ::moveit_task_constructor::Solution &msg) {
		solution_publisher_.publish(msg);
	}
	void operator()(const SolutionBase &s) { publishSolution(s); }

	// get interface state
	bool getInterfaceState(moveit_task_constructor::GetInterfaceState::Request  &req,
	                       moveit_task_constructor::GetInterfaceState::Response &res);
	// get solution
	bool getSolution(moveit_task_constructor::GetSolution::Request  &req,
	                 moveit_task_constructor::GetSolution::Response &res);
};

void publishAllPlans(const Task &task, bool wait = true);

} }
