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

		OWN_IF_MASK        = READS_INPUT | READS_OUTPUT,
		EXT_IF_MASK        = WRITES_NEXT_INPUT | WRITES_PREV_OUTPUT,
		INPUT_IF_MASK      = READS_INPUT | WRITES_PREV_OUTPUT,
		OUTPUT_IF_MASK     = READS_OUTPUT | WRITES_NEXT_INPUT,
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

	enum Direction { FORWARD = 0x01, BACKWARD = 0x02, ANYWAY = FORWARD | BACKWARD};
	void restrictDirection(Direction dir);

	virtual bool computeForward(const InterfaceState& from, planning_scene::PlanningScenePtr& to,
	                            robot_trajectory::RobotTrajectoryPtr& trajectory, double& cost);
	virtual bool computeBackward(planning_scene::PlanningScenePtr& from, const InterfaceState& to,
	                             robot_trajectory::RobotTrajectoryPtr& trajectory, double& cost);

	bool canCompute() const override;
	bool compute() override;

protected:
	// constructor for use in derived classes
	PropagatingAnyWay(PropagatingAnyWayPrivate* impl);
	void initInterface();

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
	// restrict access to backward method to provide compile-time check
	using PropagatingAnyWay::computeBackward;
};


class PropagatingBackwardPrivate;
class PropagatingBackward : public PropagatingAnyWay {
public:
	PRIVATE_CLASS(PropagatingBackward)
	PropagatingBackward(const std::string& name);

private:
	// restrict access to forward method to provide compile-time check
	using PropagatingAnyWay::computeForward;
};


class GeneratorPrivate;
class Generator : public SubTask {
public:
	PRIVATE_CLASS(Generator)
	Generator(const std::string& name);

	bool spawn(const planning_scene::PlanningSceneConstPtr &ps, double cost = 0);
};


class ConnectingPrivate;
class Connecting : public SubTask {
public:
	PRIVATE_CLASS(Connecting)
	Connecting(const std::string& name);

	// methods for manual use
	bool hasStatePair() const;
	InterfaceStatePair fetchStatePair();
	void connect(const robot_trajectory::RobotTrajectoryPtr&,
	             const InterfaceStatePair&, double cost = 0);

protected:
	virtual void newInputState(const std::list<InterfaceState>::iterator& it);
	virtual void newOutputState(const std::list<InterfaceState>::iterator& it);
};


} }
