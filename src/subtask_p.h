#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace task_constructor {

class SubTaskPrivate {
public:
	inline SubTaskPrivate(SubTask* me, const std::string& name)
	   : me_(me), name_(name)
	{}

	inline const InterfaceStateList& getBeginning() const { return beginnings_; }
	inline const InterfaceStateList& getEnd() const { return endings_; }
	inline const std::list<SubTrajectory>& getTrajectories() const { return trajectories_; }

	void addPredecessor(SubTaskPtr);
	void addSuccessor(SubTaskPtr);
	SubTrajectory& addTrajectory(const robot_trajectory::RobotTrajectoryPtr &, double cost);

	void sendForward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps);
	void sendBackward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps);

	InterfaceState* newBeginning(planning_scene::PlanningSceneConstPtr, SubTrajectory*);
	InterfaceState* newEnd(planning_scene::PlanningSceneConstPtr, SubTrajectory*);

public:
	SubTask* const me_; // pointer to owning instance
	const std::string name_;

	// avoid shared pointers back: why? to allow proper deallocation?
	SubTaskWeakPtr predecessor_;
	SubTaskPtr successor_;

	InterfaceStateList beginnings_;
	InterfaceStateList endings_;
	std::list<SubTrajectory> trajectories_;
};


class PropagatingAnyWayPrivate : public SubTaskPrivate {
	friend class PropagatingAnyWay;

public:
	inline PropagatingAnyWayPrivate(PropagatingAnyWay *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	   , it_beginnings_ (beginnings_.begin())
	   , it_endings_ (endings_.begin())
	{}

protected:
	InterfaceStateList::iterator it_beginnings_;
	InterfaceStateList::iterator it_endings_;
};


// for now, we use the same implementation for the reduced versions too
typedef PropagatingAnyWayPrivate PropagatingForwardPrivate;
typedef PropagatingAnyWayPrivate PropagatingBackwardPrivate;


class ConnectingPrivate : public SubTaskPrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name)
	   : SubTaskPrivate(me, name)
	   , it_pairs_(beginnings_.begin(), endings_.begin())
	{}

private:
	std::pair<InterfaceStateList::iterator, InterfaceStateList::iterator> it_pairs_;
};

} }
