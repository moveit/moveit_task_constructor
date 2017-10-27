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

class Introspection {
	ros::Publisher task_publisher_;
	ros::Publisher solution_publisher_;
	// TODO add services to provide InterfaceState and Solution
	ros::ServiceServer interfaceState_service_;
	ros::ServiceServer solution_service_;

	ros::NodeHandle n;

public:
	Introspection(const std::string &task_topic = "task",
	              const std::string &solution_topic = "task_plan");
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

	//advertise services
	void generateInterfaceService();
	void generateSolutionService();
	bool getInterfaceState(moveit_task_constructor::GetInterfaceState::Request  &req,
	                       moveit_task_constructor::GetInterfaceState::Response &res);
	bool getSolution(moveit_task_constructor::GetSolution::Request  &req,
	                 moveit_task_constructor::GetSolution::Response &res);
};

void publishAllPlans(const Task &task, bool wait = true);

} }
