// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "container.h"

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
	Task(Stage::pointer &&container = std::make_unique<SerialContainer>("task pipeline"));
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");

	void add(Stage::pointer &&stage);
	void clear() override;

	typedef std::function<void(const SolutionBase &s)> SolutionCallback;
	typedef std::list<SolutionCallback> SolutionCallbackList;
	/// add function to be called for every newly found solution
	SolutionCallbackList::const_iterator add(SolutionCallback &&cb);
	/// remove function callback
	void erase(SolutionCallbackList::const_iterator which);

	bool plan();
	void printState();

	size_t numSolutions() const override;

	/// function type used for processing solutions
	/// For each solution, composed from several SubTrajectories,
	/// the vector of SubTrajectories as well as the associated costs are provided.
	/// Return true, if traversal should continue, otherwise false.
	typedef std::function<bool(const SolutionTrajectory& solution,
	                           double accumulated_cost)> SolutionProcessor;
	/// process all solutions
	void processSolutions(const Task::SolutionProcessor &processor) const;

protected:
	void reset() override;
	void initModel();
	void initScene();
	bool canCompute() const override;
	bool compute() override;
	void processSolutions(const ContainerBase::SolutionProcessor &processor) const override;
	void onNewSolution(SolutionBase &s) override;

private:
	inline ContainerBase *wrapped();
	inline const ContainerBase *wrapped() const;

private:
	robot_model_loader::RobotModelLoaderPtr rml_;
	planning_scene::PlanningSceneConstPtr scene_; // initial scene
	std::list<Task::SolutionCallback> callbacks_; // functions called for each new solution

	// use separate interfaces as targets for wrapper's prevEnds() / nextStarts()
	InterfacePtr task_starts_;
	InterfacePtr task_ends_;
};

} }
