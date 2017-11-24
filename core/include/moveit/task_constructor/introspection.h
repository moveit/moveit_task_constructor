#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor_msgs/TaskDescription.h>
#include <moveit_task_constructor_msgs/TaskStatistics.h>
#include <moveit_task_constructor_msgs/Solution.h>
#include <moveit_task_constructor_msgs/GetSolution.h>

#define DESCRIPTION_TOPIC "description"
#define STATISTICS_TOPIC  "statistics"
#define SOLUTION_TOPIC    "solution"
#define GET_SOLUTION_SERVICE "get_solution"

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SolutionBase)

class IntrospectionPrivate;

/** The Introspection class provides publishing of task state and solutions.
 *
 *  It is interlinked to its task.
 */
class Introspection {
	IntrospectionPrivate *impl;

public:
	Introspection(const Task &task);
	Introspection(const Introspection &other) = delete;
	~Introspection();

	/// fill task description message for publishing the task configuration
	moveit_task_constructor_msgs::TaskDescription& fillTaskDescription(moveit_task_constructor_msgs::TaskDescription& msg);
	/// publish detailed task description
	void publishTaskDescription();

	/// fill task state message for publishing the current task state
	moveit_task_constructor_msgs::TaskStatistics& fillTaskStatistics(moveit_task_constructor_msgs::TaskStatistics& msg);
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
	bool getSolution(moveit_task_constructor_msgs::GetSolution::Request  &req,
	                 moveit_task_constructor_msgs::GetSolution::Response &res);

	/// retrieve id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage * const s) const;

	/// retrieve or set id of given solution
	uint32_t solutionId(const moveit::task_constructor::SolutionBase &s);

private:
	void fillStageStatistics(const Stage &stage, moveit_task_constructor_msgs::StageStatistics &s);
	void fillSolution(moveit_task_constructor_msgs::Solution &msg, const SolutionBase &s);
	/// retrieve or set id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage * const s);
};

} }
