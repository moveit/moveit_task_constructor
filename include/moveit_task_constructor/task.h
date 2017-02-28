// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <vector>

namespace moveit::core {
	MOVEIT_CLASS_FORWARD(RobotModel);
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

protected:
	void addSubTask( SubTaskPtr );

	std::vector<SubTaskPtr> subtasks_;

	robot_model_loader::RobotModelLoaderPtr rml_;
};

}
