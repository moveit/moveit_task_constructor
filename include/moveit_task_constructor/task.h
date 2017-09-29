// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/storage.h>

#include <moveit/macros/class_forward.h>

#include <ros/ros.h>

#include <vector>

namespace planning_scene {
	MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace robot_model_loader {
	MOVEIT_CLASS_FORWARD(RobotModelLoader)
}

namespace planning_pipeline {
	MOVEIT_CLASS_FORWARD(PlanningPipeline)
}

namespace moveit { namespace core {
	MOVEIT_CLASS_FORWARD(RobotState)
}}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(SubTask)
MOVEIT_CLASS_FORWARD(Task)

class Task {
public:
	typedef std::function<bool(std::vector<SubTrajectory*>&)> SolutionCallback;

	Task();
	~Task();

	void add( SubTaskPtr );

	bool plan();

	bool processSolutions(const SolutionCallback &processor) const;

	const moveit::core::RobotState &getCurrentRobotState() const;

	void printState();

	void clear();

protected:
	std::vector<SubTaskPtr> subtasks_;

	planning_scene::PlanningScenePtr scene_;
	robot_model_loader::RobotModelLoaderPtr rml_;

	planning_pipeline::PlanningPipelinePtr planner_;
};

} }
