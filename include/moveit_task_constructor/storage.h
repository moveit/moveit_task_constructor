// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/planning_scene/planning_scene.h>

namespace moveit::task_constructor {

MOVEIT_CLASS_FORWARD(SubTrajectory);
MOVEIT_CLASS_FORWARD(InterfaceState);

struct SubTrajectory {
	robot_trajectory::RobotTrajectoryPtr trajectory;
	std::vector<InterfaceState*> next;
	std::vector<InterfaceState*> prev;
};

struct InterfaceState {
	SubTrajectory* connection;

	planning_scene::PlanningSceneConstPtr state;
};

}
