// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <vector>
#include <cassert>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory);
}

namespace moveit::task_constructor {

MOVEIT_CLASS_FORWARD(SubTrajectory);
MOVEIT_CLASS_FORWARD(InterfaceState);

struct InterfaceState {
	InterfaceState(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* previous, SubTrajectory* next)
		: state(ps),
		  previous_trajectory(previous),
		  next_trajectory(next)
	{}

	SubTrajectory* previous_trajectory;
	SubTrajectory* next_trajectory;

	planning_scene::PlanningSceneConstPtr state;
};

struct SubTrajectory {
	SubTrajectory(robot_trajectory::RobotTrajectoryPtr traj)
		: trajectory(traj)
	{}

	void connectToBeginning(InterfaceState& state){
		begin.push_back(&state);
		assert(state.next_trajectory == NULL);
		state.next_trajectory= this;
	}

	robot_trajectory::RobotTrajectoryPtr trajectory;
	std::vector<InterfaceState*> begin;
	std::vector<InterfaceState*> end;
};

}
