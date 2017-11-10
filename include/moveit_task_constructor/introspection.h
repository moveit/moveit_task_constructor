#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor/TaskDescription.h>
#include <moveit_task_constructor/TaskStatistics.h>
#include <moveit_task_constructor/Solution.h>
#include <moveit_task_constructor/GetSolution.h>

#define DESCRIPTION_TOPIC "description"
#define STATISTICS_TOPIC  "statistics"
#define SOLUTION_TOPIC    "solution"

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
	moveit_task_constructor::TaskDescription& fillTaskDescription(moveit_task_constructor::TaskDescription& msg);
	/// publish detailed task description
	void publishTaskDescription();

	/// fill task state message for publishing the current task state
	moveit_task_constructor::TaskStatistics& fillTaskStatistics(moveit_task_constructor::TaskStatistics& msg);
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

	/// retrieve id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage * const s) const;

	/// retrieve or set id of given solution
	uint32_t solutionId(const moveit::task_constructor::SolutionBase &s);

private:
	void fillStageStatistics(const Stage &stage, moveit_task_constructor::StageStatistics &s);
	/// retrieve or set id of given stage
	uint32_t stageId(const moveit::task_constructor::Stage * const s);
};

} }
