// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/storage.h>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>

#include <vector>
#include <list>
#include <tuple>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline)
}

namespace moveit { namespace task_constructor {

typedef std::list<InterfaceState> InterfaceStateList;
typedef std::pair<InterfaceState&, InterfaceState&> InterfaceStatePair;

MOVEIT_CLASS_FORWARD(SubTask)
typedef std::weak_ptr<SubTask> SubTaskWeakPtr;

class Task;
class SubTaskPrivate;
class SubTask {
	friend class SubTaskPrivate; // allow access to impl_
	friend class Task; // TODO: remove when we have SubTaskContainers

protected:
	SubTaskPrivate* const impl_;

public:
	enum PropagationType { FORWARD, BACKWARD, ANYWAY, GENERATOR, CONNECTING };

	~SubTask();
	const std::string& getName() const;

	virtual bool canCompute() = 0;
	virtual bool compute() = 0;

	void setPlanningScene(planning_scene::PlanningSceneConstPtr);
	void setPlanningPipeline(planning_pipeline::PlanningPipelinePtr);

protected:
	/// can only instantiated by derived classes
	SubTask(SubTaskPrivate *impl);

	/// methods called when a new InterfaceState was spawned
	virtual inline void newBeginning(const InterfaceStateList::iterator& it) {}
	virtual inline void newEnd(const InterfaceStateList::iterator& it) {}

	planning_scene::PlanningSceneConstPtr scene_;
	planning_pipeline::PlanningPipelinePtr planner_;
};


class PropagatingAnyWayPrivate;
class PropagatingAnyWay : public SubTask {
public:
	PropagatingAnyWay(const std::string& name);
	static inline constexpr PropagationType type() { return ANYWAY; }

	bool hasBeginning() const;
	InterfaceState& fetchStateBeginning();
	void sendForward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                 const InterfaceState& from,
	                 const planning_scene::PlanningSceneConstPtr& to,
	                 double cost = 0);

	bool hasEnding() const;
	InterfaceState& fetchStateEnding();
	void sendBackward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                  const planning_scene::PlanningSceneConstPtr& from,
	                  const InterfaceState& to,
	                  double cost = 0);

protected:
	// constructor for use in derived classes
	inline PropagatingAnyWay(PropagatingAnyWayPrivate* impl);

	// get informed when new beginnings and endings become available
	void newBeginning(const InterfaceStateList::iterator& it);
	void newEnd(const InterfaceStateList::iterator& it);
};


class PropagatingForward : public PropagatingAnyWay {
public:
	PropagatingForward(const std::string& name);
	static inline constexpr PropagationType type() { return FORWARD; }

private:
	// restrict access to backward methods
	using PropagatingAnyWay::hasEnding;
	using PropagatingAnyWay::fetchStateEnding;
	using PropagatingAnyWay::sendBackward;
	// don't care about new endings
	inline void newEnd(const InterfaceStateList::iterator& it) {}
};


class PropagatingBackward : public PropagatingAnyWay {
public:
	PropagatingBackward(const std::string& name);
	static inline constexpr PropagationType type() { return BACKWARD; }

private:
	// restrict access to forward methods
	using PropagatingAnyWay::hasBeginning;
	using PropagatingAnyWay::fetchStateBeginning;
	using PropagatingAnyWay::sendForward;
	// don't care about new beginnings
	inline void newBeginning(const InterfaceStateList::iterator& it) {}
};


class Generator : public SubTask {
public:
	Generator(const std::string& name);
	static inline constexpr PropagationType type() { return GENERATOR; }

	void spawn(const planning_scene::PlanningSceneConstPtr &ps, double cost = 0);
};


class Connecting : public SubTask {
public:
	Connecting(const std::string& name);
	static inline constexpr PropagationType type() { return CONNECTING; }

	bool hasStatePair() const;
	InterfaceStatePair fetchStatePair();
	void connect(const robot_trajectory::RobotTrajectoryPtr&, const InterfaceStatePair&, double cost = 0);

protected:
	virtual void newBeginning(const InterfaceStateList::iterator& it);
	virtual void newEnd(const InterfaceStateList::iterator& it);
};


} }
