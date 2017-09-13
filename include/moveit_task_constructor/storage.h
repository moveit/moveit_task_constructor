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

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(SubTrajectory);
MOVEIT_CLASS_FORWARD(InterfaceState);

struct InterfaceState {
	InterfaceState(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* previous, SubTrajectory* next)
		: state(ps)
	{
		if( previous )
			previous_trajectory.push_back(previous);
		if( next )
			next_trajectory.push_back(next);
	}

	std::vector<SubTrajectory*> previous_trajectory;
	std::vector<SubTrajectory*> next_trajectory;

	planning_scene::PlanningSceneConstPtr state;
};

struct SubTrajectory {
	SubTrajectory(robot_trajectory::RobotTrajectoryPtr traj)
		: trajectory(traj),
		  begin(NULL),
		  end(NULL),
		  flag(false)
	{}

	void hasBeginning(InterfaceState& state){
		assert(begin == NULL);
		begin= &state;
		state.next_trajectory.push_back(this);
	}

	void hasEnding(InterfaceState& state){
		assert(end == NULL);
		end= &state;
		state.previous_trajectory.push_back(this);
	}

	robot_trajectory::RobotTrajectoryPtr trajectory;
	InterfaceState* begin;
	InterfaceState* end;

	bool flag;
};

} }
