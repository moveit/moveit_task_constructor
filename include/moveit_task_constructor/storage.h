// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/macros/class_forward.h>

#include <deque>
#include <list>
#include <cassert>
#include <functional>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(SubTrajectory)
MOVEIT_CLASS_FORWARD(InterfaceState)
MOVEIT_CLASS_FORWARD(Interface)
MOVEIT_CLASS_FORWARD(SubTask)


/** InterfaceState describes a potential start or goal state for a planning stage.
 *
 *  A start or goal state for planning is essentially defined by the state of a planning scene.
 */
class InterfaceState {
	friend class SubTrajectory;

public:
	typedef std::deque<SubTrajectory*> SubTrajectoryList;

	InterfaceState(const planning_scene::PlanningSceneConstPtr& ps)
		: state(ps)
	{}

	inline const SubTrajectoryList& incomingTrajectories() const { return incoming_trajectories_; }
	inline const SubTrajectoryList& outgoingTrajectories() const { return outgoing_trajectories_; }

public:
	planning_scene::PlanningSceneConstPtr state;

private:
	mutable SubTrajectoryList incoming_trajectories_;
	mutable SubTrajectoryList outgoing_trajectories_;
};


/** Interface provides a list of InterfaceStates available as input for a stage.
 *
 *  This is essentially an adaptor to a container class, to allow for notification
 *  of the interface's owner when new states become available
 */
class Interface : protected std::list<InterfaceState> {
public:
	typedef std::list<InterfaceState> container_type;
	typedef std::function<void(const container_type::iterator&)> NotifyFunction;
	Interface(const NotifyFunction &notify);
	// add a new InterfaceState, connect the trajectory (either incoming or outgoing) to the newly created state
	// and finally run the notify callback
	container_type::iterator add(const planning_scene::PlanningSceneConstPtr& ps, SubTrajectory* incoming, SubTrajectory* outgoing);

	using container_type::value_type;
	using container_type::reference;
	using container_type::const_reference;

	using container_type::iterator;
	using container_type::const_iterator;
	using container_type::reverse_iterator;
	using container_type::const_reverse_iterator;

	using container_type::empty;
	using container_type::size;
	using container_type::clear;
	using container_type::front;
	using container_type::back;

	using container_type::begin;
	using container_type::cbegin;
	using container_type::end;
	using container_type::cend;
	using container_type::rbegin;
	using container_type::crbegin;
	using container_type::rend;
	using container_type::crend;

private:
	const NotifyFunction notify_;
};


struct SubTrajectory {
	SubTrajectory(robot_trajectory::RobotTrajectoryPtr traj)
		: trajectory(traj),
		  begin(NULL),
		  end(NULL),
		  cost(0)
	{}

	inline void setStartState(const InterfaceState& state){
		assert(begin == NULL);
		begin= &state;
		state.outgoing_trajectories_.push_back(this);
	}

	inline void setEndState(const InterfaceState& state){
		assert(end == NULL);
		end= &state;
		state.incoming_trajectories_.push_back(this);
	}

	// TODO: trajectories could have a name, e.g. a generator could name its solutions
	const robot_trajectory::RobotTrajectoryPtr trajectory;
	const InterfaceState* begin;
	const InterfaceState* end;
	double cost;
};

} }
