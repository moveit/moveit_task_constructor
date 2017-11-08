// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "container.h"

#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor/TaskDescription.h>
#include <moveit_task_constructor/TaskStatistics.h>
#include <moveit_task_constructor/Solution.h>

namespace robot_model_loader {
	MOVEIT_CLASS_FORWARD(RobotModelLoader)
}
namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotModel)
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(ContainerBase)
MOVEIT_CLASS_FORWARD(Task)

/** A Task is the root of a tree of stages.
 *
 * Actually a tasks wraps a single container, which serves as the root of all stages.
 * The wrapped container spawns its solutions into the prevEnds(), nextStarts() interfaces,
 * which are provided by the wrappers end_ and start_ and interfaces respectively. */
class Task : protected WrapperBase {
public:
	Task(Stage::pointer &&container = std::make_unique<SerialContainer>("task pipeline"));
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");

	std::string id() const;
	void add(Stage::pointer &&stage);
	void clear() override;

	typedef std::function<void(const SolutionBase &s)> SolutionCallback;
	typedef std::list<SolutionCallback> SolutionCallbackList;
	/// add function to be called for every newly found solution
	SolutionCallbackList::const_iterator addSolutionCallback(SolutionCallback &&cb);
	/// remove function callback
	void erase(SolutionCallbackList::const_iterator which);

	typedef std::function<void(const Task &t)> TaskCallback;
	typedef std::list<TaskCallback> TaskCallbackList;
	/// add function to be called for every newly found solution
	TaskCallbackList::const_iterator addTaskCallback(TaskCallback &&cb);
	/// remove function callback
	void erase(TaskCallbackList::const_iterator which);

	bool plan();
	/// print current state std::cout
	static void printState(const Task &t);
	/// fill task description message for publishing the task configuration
	moveit_task_constructor::TaskDescription& fillTaskDescription(moveit_task_constructor::TaskDescription& msg) const;
	/// fill task state message for publishing the current task state
	moveit_task_constructor::TaskStatistics& fillTaskStatistics(moveit_task_constructor::TaskStatistics& msg) const;

	size_t numSolutions() const override;

	/// function type used for processing solutions
	/// For each solution, composed from several SubTrajectories,
	/// the vector of SubTrajectories as well as the associated costs are provided.
	/// Return true, if traversal should continue, otherwise false.
	typedef std::function<bool(const ::moveit_task_constructor::Solution& msg,
	                           double accumulated_cost)> SolutionProcessor;
	/// process all solutions
	void processSolutions(const Task::SolutionProcessor &processor) const;

	/// access stage tree
	ContainerBase *stages();
	const ContainerBase *stages() const;

protected:
	void reset() override;
	void initModel();
	void initScene();
	bool canCompute() const override;
	bool compute() override;
	void processSolutions(const ContainerBase::SolutionProcessor &processor) const override;
	void onNewSolution(SolutionBase &s) override;

private:
	size_t id_; // unique task ID
	robot_model_loader::RobotModelLoaderPtr rml_;
	planning_scene::PlanningSceneConstPtr scene_; // initial scene
	std::list<Task::SolutionCallback> solution_cbs_; // functions called for each new solution
	std::list<Task::TaskCallback> task_cbs_; // functions to monitor task's planning progress

	// use separate interfaces as targets for wrapper's prevEnds() / nextStarts()
	InterfacePtr task_starts_;
	InterfacePtr task_ends_;
};

} }
