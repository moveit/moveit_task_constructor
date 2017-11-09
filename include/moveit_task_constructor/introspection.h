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

/** The Introspection class provides publishing of task state and solutions.
 *
 *  It is interlinked to its task.
 */
class Introspection {
	ros::NodeHandle nh_;
	/// associated task
	const Task &task_;

	/// publish task detailed description and current state
	ros::Publisher task_description_publisher_;
	ros::Publisher task_statistics_publisher_;
	/// publish new solutions
	ros::Publisher solution_publisher_;
	/// services to provide an individual Solution
	ros::ServiceServer get_solution_service_;

public:
	Introspection(const Task &task);
	Introspection(const Introspection &other) = delete;

	/// publish detailed task description
	void publishTaskDescription();

	/// publish the current state of task
	void publishTaskState();

	/// indicate that this task was reset
	void reset();

	/// publish the given solution
	void publishSolution(const SolutionBase &s);
	/// operator version for use in stage callbacks
	void operator()(const SolutionBase &s) { publishSolution(s); }

	/// publish all top-level solutions of task
	void publishAllSolutions(bool wait = true);

	/// get solution
	bool getSolution(moveit_task_constructor::GetSolution::Request  &req,
	                 moveit_task_constructor::GetSolution::Response &res);
};

} }
