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


/// exception thrown by Stage::init()
/// It collects individual errors in stages throughout the pipeline to allow overall error reporting
class InitStageException : public std::exception {
	friend std::ostream& operator<<(std::ostream &os, const InitStageException& e);

public:
	explicit InitStageException() {}
	explicit InitStageException(const Stage& stage, const std::string& msg) {
		push_back(stage, msg);
	}

	/// push_back a single new error in stage
	void push_back(const Stage& stage, const std::string& msg);
	/// append all the errors from other (which is emptied)
	void append(InitStageException &other);

	/// check of existing errors
	operator bool() const { return !errors_.empty(); }

	virtual const char* what() const noexcept override;
private:
	std::list<std::pair<const Stage*, const std::string>> errors_;
};
std::ostream& operator<<(std::ostream &os, const InitStageException& e);


class StagePrivate;
class Stage {
	friend std::ostream& operator<<(std::ostream &os, const Stage& stage);

public:
	PRIVATE_CLASS(Stage)
	typedef std::unique_ptr<Stage> pointer;
	~Stage();

	/// auto-convert Stage to StagePrivate* when needed
	operator StagePrivate*();
	operator const StagePrivate*() const;

	/// reset stage, clearing all solutions, interfaces, etc.
	virtual void reset();
	/// initialize stage once before planning
	virtual void init(const planning_scene::PlanningSceneConstPtr& scene);

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
	void reset() override;
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

	virtual void init(const planning_scene::PlanningSceneConstPtr &scene) override;
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
