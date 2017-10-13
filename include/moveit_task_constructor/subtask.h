// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "utils.h"
#include <moveit/macros/class_forward.h>
#include <vector>
#include <list>

#define PRIVATE_CLASS(Class) \
	friend class Class##Private; \
	inline const Class##Private* pimpl() const; \
	inline Class##Private* pimpl();

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

public:
	PRIVATE_CLASS(SubTask)
	typedef std::unique_ptr<SubTask> pointer;

	~SubTask();

	/// initialize stage once before planning
	virtual bool init(const planning_scene::PlanningSceneConstPtr& scene);
	const std::string& getName() const;

protected:
	/// SubTask can only be instantiated through derived classes
	SubTask(SubTaskPrivate *impl);

protected:
	// TODO: use unique_ptr<SubTaskPrivate> and get rid of destructor
	SubTaskPrivate* const pimpl_; // constness guarantees one initial write
};
std::ostream& operator<<(std::ostream &os, const SubTask& stage);


class PropagatingEitherWayPrivate;
class PropagatingEitherWay : public SubTask {
public:
	PRIVATE_CLASS(PropagatingEitherWay)
	PropagatingEitherWay(const std::string& name);

	enum Direction { FORWARD = 0x01, BACKWARD = 0x02, ANYWAY = FORWARD | BACKWARD};
	void restrictDirection(Direction dir);

	virtual bool computeForward(const InterfaceState& from) = 0;
	virtual bool computeBackward(const InterfaceState& to) = 0;

	void sendForward(const InterfaceState& from,
	                 const planning_scene::PlanningSceneConstPtr& to,
	                 const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                 double cost = 0);
	void sendBackward(const planning_scene::PlanningSceneConstPtr& from,
	                  const InterfaceState& to,
	                  const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                  double cost = 0);

protected:
	// constructor for use in derived classes
	PropagatingEitherWay(PropagatingEitherWayPrivate* impl);
	void initInterface();
};


class PropagatingForwardPrivate;
class PropagatingForward : public PropagatingEitherWay {
public:
	PRIVATE_CLASS(PropagatingForward)
	PropagatingForward(const std::string& name);

private:
	// restrict access to backward method to provide compile-time check
	bool computeBackward(const InterfaceState &to) override;
	using PropagatingEitherWay::sendBackward;
};


class PropagatingBackwardPrivate;
class PropagatingBackward : public PropagatingEitherWay {
public:
	PRIVATE_CLASS(PropagatingBackward)
	PropagatingBackward(const std::string& name);

private:
	// restrict access to forward method to provide compile-time check
	bool computeForward(const InterfaceState &from) override;
	using PropagatingEitherWay::sendForward;
};


class GeneratorPrivate;
class Generator : public SubTask {
public:
	PRIVATE_CLASS(Generator)
	Generator(const std::string& name);

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;
	void spawn(const planning_scene::PlanningSceneConstPtr &ps, double cost = 0);
};


class ConnectingPrivate;
class Connecting : public SubTask {
public:
	PRIVATE_CLASS(Connecting)
	Connecting(const std::string& name);

	virtual bool compute(const InterfaceState& from, const InterfaceState& to) = 0;
	void connect(const InterfaceState& from, const InterfaceState& to,
	             const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost = 0);
};


} }
