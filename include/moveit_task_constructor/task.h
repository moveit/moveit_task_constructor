// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "container.h"

#include <moveit/macros/class_forward.h>

namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotModel)
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Stage)
MOVEIT_CLASS_FORWARD(ContainerBase)
MOVEIT_CLASS_FORWARD(Task)

class TaskPrivate;
class Task : protected WrapperBase {
	PRIVATE_CLASS(Task)
public:
	Task(Stage::pointer &&container = std::make_unique<SerialContainer>("task pipeline"));
	static planning_pipeline::PlanningPipelinePtr createPlanner(const moveit::core::RobotModelConstPtr &model,
	                                                            const std::string &ns = "move_group",
	                                                            const std::string &planning_plugin_param_name = "planning_plugin",
	                                                            const std::string &adapter_plugins_param_name = "request_adapters");

	void add(Stage::pointer &&stage);
	void clear();

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

	static SolutionTrajectory& flatten(const SolutionBase &s, SolutionTrajectory& result);

protected:
	void init(const planning_scene::PlanningSceneConstPtr &scene) override;
	bool canCompute() const override;
	bool compute() override;
	void processSolutions(const ContainerBase::SolutionProcessor &processor) const override;
	void append(const SolutionBase& s, SolutionTrajectory& solution) const override;
	void onNewSolution(SolutionBase &s) override;

private:
	inline ContainerBase *wrapped();
	inline const ContainerBase *wrapped() const;
};

} }
