// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <vector>
#include <cassert>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(SubTrajectory)
MOVEIT_CLASS_FORWARD(InterfaceState)

class InterfaceState {
	friend class SubTrajectory;

public:
	typedef std::vector<SubTrajectory*> SubTrajectoryList;

	InterfaceState(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* previous, SubTrajectory* next)
		: state(ps)
	{
		if( previous )
			previous_trajectories_.push_back(previous);
		if( next )
			next_trajectories_.push_back(next);
	}

	inline const SubTrajectoryList& previousTrajectories() const { return previous_trajectories_; }
	inline const SubTrajectoryList& nextTrajectories() const { return next_trajectories_; }

public:
	planning_scene::PlanningSceneConstPtr state;
	double cost; // minimal costs

private:
	mutable SubTrajectoryList previous_trajectories_;
	mutable SubTrajectoryList next_trajectories_;
};

struct SubTrajectory {
	SubTrajectory(robot_trajectory::RobotTrajectoryPtr traj)
		: trajectory(traj),
		  begin(NULL),
		  end(NULL),
		  cost(0)
	{}

	void setBeginning(const InterfaceState& state){
		assert(begin == NULL);
		begin= &state;
		state.next_trajectories_.push_back(this);
	}

	void setEnding(const InterfaceState& state){
		assert(end == NULL);
		end= &state;
		state.previous_trajectories_.push_back(this);
	}

	robot_trajectory::RobotTrajectoryPtr trajectory;
	const InterfaceState* begin;
	const InterfaceState* end;
	double cost;
};

} }
