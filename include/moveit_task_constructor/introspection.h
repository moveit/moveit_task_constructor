#pragma once

#include <moveit/macros/class_forward.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <moveit_task_constructor/TaskDescription.h>
#include <moveit_task_constructor/TaskStatistics.h>
#include <moveit_task_constructor/Solution.h>
#include <moveit_task_constructor/GetSolution.h>
#include <boost/bimap.hpp>

#define DESCRIPTION_TOPIC "description"
#define STATISTICS_TOPIC  "statistics"
#define SOLUTION_TOPIC    "solution"

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
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

	/// mapping from stages to their id
	std::map<const void*, moveit_task_constructor::StageStatistics::_id_type> stage_to_id_map_;
	boost::bimap<uint32_t, const SolutionBase*> id_solution_bimap_;

public:
	Introspection(const Task &task);
	Introspection(const Introspection &other) = delete;

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
	void publishAllSolutions(bool wait = true) const;

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
