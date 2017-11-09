// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "container.h"

#include <moveit_task_constructor/introspection.h>

#include <moveit_task_constructor/TaskDescription.h>
#include <moveit_task_constructor/TaskStatistics.h>
#include <moveit_task_constructor/Solution.h>

#include <moveit/macros/class_forward.h>

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
	Task(const std::string& id = "",
        Stage::pointer &&container = std::make_unique<SerialContainer>("task pipeline"));
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");
	~Task();

	std::string id() const;

	void add(Stage::pointer &&stage);
	void clear() override;

	/// enable introspection publishing for use with rviz
	void enableIntrospection(bool enable = true);
	Introspection &introspection();

	typedef std::function<void(const Task &t)> TaskCallback;
	typedef std::list<TaskCallback> TaskCallbackList;
	/// add function to be called after each top-level iteration
	TaskCallbackList::const_iterator addTaskCallback(TaskCallback &&cb);
	/// remove function callback
	void erase(TaskCallbackList::const_iterator which);

	void reset() override;
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;

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

	/// publish all top-level solutions
	void publishAllSolutions(bool wait = true);

	/// access stage tree
	ContainerBase *stages();
	const ContainerBase *stages() const;

protected:
	void initModel();
	void initScene();
	bool canCompute() const override;
	bool compute() override;
	void processSolutions(const ContainerBase::SolutionProcessor &processor) const override;
	void onNewSolution(SolutionBase &s) override;

private:
	std::string id_;
	robot_model_loader::RobotModelLoaderPtr rml_;
	planning_scene::PlanningSceneConstPtr scene_; // initial scene

	// use separate interfaces as targets for wrapper's prevEnds() / nextStarts()
	InterfacePtr task_starts_;
	InterfacePtr task_ends_;

	// introspection and monitoring
	std::unique_ptr<Introspection> introspection_;
	std::list<Task::TaskCallback> task_cbs_; // functions to monitor task's planning progress
};

} }
