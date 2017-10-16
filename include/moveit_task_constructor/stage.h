// copyright Michael 'v4hn' Goerner @ 2017
// copyright Robert Haschke @ 2017

#pragma once

#include "utils.h"
#include <moveit/macros/class_forward.h>
#include <moveit_task_constructor/storage.h>
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
MOVEIT_CLASS_FORWARD(Stage)
class InterfaceState;
typedef std::pair<const InterfaceState&, const InterfaceState&> InterfaceStatePair;


// SubTrajectory connects interface states of ComputeStages
class SubTrajectory : public SolutionBase {
public:
	explicit SubTrajectory(StagePrivate* creator, const robot_trajectory::RobotTrajectoryPtr& traj, double cost);

	robot_trajectory::RobotTrajectoryConstPtr trajectory() const { return trajectory_; }
	const std::string& name() const { return name_; }
	void setName(const std::string& name) { name_ = name; }

private:
	const robot_trajectory::RobotTrajectoryPtr trajectory_;
	// trajectories could have a name, e.g. a generator could name its solutions
	std::string name_;
};


class StagePrivate;
class Stage {
	friend std::ostream& operator<<(std::ostream &os, const Stage& stage);

public:
	PRIVATE_CLASS(Stage)
	typedef std::unique_ptr<Stage> pointer;
	~Stage();

	/// initialize stage once before planning
	virtual bool init(const planning_scene::PlanningSceneConstPtr& scene);

	const std::string& name() const;
	virtual size_t numSolutions() const = 0;

protected:
	/// Stage can only be instantiated through derived classes
	Stage(StagePrivate *impl);

protected:
	StagePrivate* const pimpl_; // constness guarantees one initial write
};
std::ostream& operator<<(std::ostream &os, const Stage& stage);


class ComputeBasePrivate;
class ComputeBase : public Stage {
public:
	PRIVATE_CLASS(ComputeBase)
	virtual size_t numSolutions() const override;

protected:
	/// ComputeBase can only be instantiated by derived classes in stage.cpp
	ComputeBase(ComputeBasePrivate* impl);

	// TODO: Do we really need/want to expose the trajectories?
	const std::list<SubTrajectory>& trajectories() const;
	SubTrajectory& addTrajectory(const robot_trajectory::RobotTrajectoryPtr &, double cost);
};


class PropagatingEitherWayPrivate;
class PropagatingEitherWay : public ComputeBase {
public:
	PRIVATE_CLASS(PropagatingEitherWay)
	PropagatingEitherWay(const std::string& name);

	enum Direction { FORWARD = 0x01, BACKWARD = 0x02, ANYWAY = FORWARD | BACKWARD};
	void restrictDirection(Direction dir);

	virtual bool init(const planning_scene::PlanningSceneConstPtr &scene) override;
	virtual bool computeForward(const InterfaceState& from) = 0;
	virtual bool computeBackward(const InterfaceState& to) = 0;

	void sendForward(const InterfaceState& from,
	                 InterfaceState&& to,
	                 const robot_trajectory::RobotTrajectoryPtr& trajectory,
	                 double cost = 0);
	void sendBackward(InterfaceState&& from,
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
class Generator : public ComputeBase {
public:
	PRIVATE_CLASS(Generator)
	Generator(const std::string& name);

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;
	void spawn(InterfaceState &&state, double cost = 0);
};


class ConnectingPrivate;
class Connecting : public ComputeBase {
public:
	PRIVATE_CLASS(Connecting)
	Connecting(const std::string& name);

	virtual bool compute(const InterfaceState& from, const InterfaceState& to) = 0;
	void connect(const InterfaceState& from, const InterfaceState& to,
	             const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost = 0);
};


} }
