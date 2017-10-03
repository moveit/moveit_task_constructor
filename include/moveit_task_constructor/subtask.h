// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "utils.h"
#include <moveit/macros/class_forward.h>
#include <vector>
#include <list>

#define PRIVATE_CLASS(Class) \
	friend class Class##Private;

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Interface)
MOVEIT_CLASS_FORWARD(SubTask)
MOVEIT_CLASS_FORWARD(SubTrajectory)
class InterfaceState;
typedef std::pair<const InterfaceState&, const InterfaceState&> InterfaceStatePair;


class SubTaskPrivate;
class SubTask {
	friend std::ostream& operator<<(std::ostream &os, const SubTask& stage);
	friend class SubTaskPrivate;

public:
	inline const SubTaskPrivate* pimpl_func() const { return pimpl_; }
	inline SubTaskPrivate* pimpl_func() { return pimpl_; }

	typedef std::unique_ptr<SubTask> pointer;
	enum InterfaceFlag {
		READS_INPUT        = 0x01,
		READS_OUTPUT       = 0x02,
		WRITES_NEXT_INPUT  = 0x04,
		WRITES_PREV_OUTPUT = 0x08,
		WRITES_UNKNOWN     = 0x10,

		OWN_IF_MASK        = READS_INPUT | READS_OUTPUT,
		EXT_IF_MASK        = WRITES_NEXT_INPUT | WRITES_PREV_OUTPUT,
		INPUT_IF_MASK      = READS_INPUT | WRITES_PREV_OUTPUT | WRITES_UNKNOWN,
		OUTPUT_IF_MASK     = READS_OUTPUT | WRITES_NEXT_INPUT | WRITES_UNKNOWN,
	};
	typedef Flags<InterfaceFlag> InterfaceFlags;
	InterfaceFlags interfaceFlags() const;

	~SubTask();
	const std::string& getName() const;


	// TODO: results from { TIMEOUT, EXHAUSTED, FINISHED, WAITING }
	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

	planning_scene::PlanningSceneConstPtr scene() const;
	planning_pipeline::PlanningPipelinePtr planner() const;
	void setPlanningScene(const planning_scene::PlanningSceneConstPtr&);
	void setPlanningPipeline(const planning_pipeline::PlanningPipelinePtr&);

protected:
	/// can only instantiated by derived classes
	SubTask(SubTaskPrivate *impl);

	/// methods called when a new InterfaceState was spawned
	virtual void newInputState(const std::list<InterfaceState>::iterator&) {}
	virtual void newOutputState(const std::list<InterfaceState>::iterator&) {}

protected:
	// TODO: use unique_ptr<SubTaskPrivate> and get rid of destructor
	SubTaskPrivate* const pimpl_; // constness guarantees one initial write
};
std::ostream& operator<<(std::ostream &os, const SubTask& stage);


class PropagatingAnyWayPrivate;
class PropagatingAnyWay : public SubTask {
public:
	PRIVATE_CLASS(PropagatingAnyWay)
	PropagatingAnyWay(const std::string& name);

	bool hasBeginning() const;
	const InterfaceState &fetchStateBeginning();
	void sendForward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                 const InterfaceState& from,
	                 const planning_scene::PlanningSceneConstPtr& to,
	                 double cost = 0);

	bool hasEnding() const;
	const InterfaceState &fetchStateEnding();
	void sendBackward(const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                  const planning_scene::PlanningSceneConstPtr& from,
	                  const InterfaceState& to,
	                  double cost = 0);

protected:
	// constructor for use in derived classes
	PropagatingAnyWay(PropagatingAnyWayPrivate* impl);

	// get informed when new beginnings and endings become available
	void newInputState(const std::list<InterfaceState>::iterator& it);
	void newOutputState(const std::list<InterfaceState>::iterator& it);
};


class PropagatingForwardPrivate;
class PropagatingForward : public PropagatingAnyWay {
public:
	PRIVATE_CLASS(PropagatingForward)
	PropagatingForward(const std::string& name);

private:
	// restrict access to backward methods
	using PropagatingAnyWay::hasEnding;
	using PropagatingAnyWay::fetchStateEnding;
	using PropagatingAnyWay::sendBackward;
};


class PropagatingBackwardPrivate;
class PropagatingBackward : public PropagatingAnyWay {
public:
	PRIVATE_CLASS(PropagatingBackward)
	PropagatingBackward(const std::string& name);

private:
	// restrict access to forward methods
	using PropagatingAnyWay::hasBeginning;
	using PropagatingAnyWay::fetchStateBeginning;
	using PropagatingAnyWay::sendForward;
};


class GeneratorPrivate;
class Generator : public SubTask {
public:
	PRIVATE_CLASS(Generator)
	Generator(const std::string& name);

	void spawn(const planning_scene::PlanningSceneConstPtr &ps, double cost = 0);
};


class ConnectingPrivate;
class Connecting : public SubTask {
public:
	PRIVATE_CLASS(Connecting)
	Connecting(const std::string& name);

	bool hasStatePair() const;
	InterfaceStatePair fetchStatePair();
	void connect(const robot_trajectory::RobotTrajectoryPtr&, const InterfaceStatePair&, double cost = 0);

protected:
	virtual void newInputState(const std::list<InterfaceState>::iterator& it);
	virtual void newOutputState(const std::list<InterfaceState>::iterator& it);
};


} }
