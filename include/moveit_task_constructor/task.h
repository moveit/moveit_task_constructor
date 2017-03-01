// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <vector>

namespace planning_scene {
	MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace robot_model_loader {
	MOVEIT_CLASS_FORWARD(RobotModelLoader);
}

namespace moveit::task_constructor {

MOVEIT_CLASS_FORWARD(SubTask);


MOVEIT_CLASS_FORWARD(Task);

class Task {
public:
	Task();

	void addStart( SubTaskPtr );
	void addAfter( SubTaskPtr );

	bool plan();

	void printState();

protected:
	void addSubTask( SubTaskPtr );

	std::vector<SubTaskPtr> subtasks_;

	planning_scene::PlanningScenePtr scene_;
	robot_model_loader::RobotModelLoaderPtr rml_;
};

}
