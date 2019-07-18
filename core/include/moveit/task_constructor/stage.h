/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
 *  Copyright (c) 2017, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael 'v4hn' Goerner, Robert Haschke
   Desc:   Define the Stage interfaces as the basis for any custom Stage
*/

#pragma once

#include "utils.h"
#include <moveit/macros/class_forward.h>
#include <moveit/task_constructor/storage.h>
#include <vector>
#include <list>

#define PRIVATE_CLASS(Class)                   \
	friend class Class##Private;                \
	inline const Class##Private* pimpl() const; \
	inline Class##Private* pimpl();

namespace moveit {
namespace core {
MOVEIT_CLASS_FORWARD(RobotModel)
}
}

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene)
}

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline)
}

namespace robot_trajectory {
MOVEIT_CLASS_FORWARD(RobotTrajectory)
}

namespace moveit {
namespace task_constructor {

enum InterfaceFlag
{
	READS_START = 0x01,
	READS_END = 0x02,
	WRITES_NEXT_START = 0x04,
	WRITES_PREV_END = 0x08,

	CONNECT = READS_START | READS_END,
	PROPAGATE_FORWARDS = READS_START | WRITES_NEXT_START,
	PROPAGATE_BACKWARDS = READS_END | WRITES_PREV_END,
	GENERATE = WRITES_PREV_END | WRITES_NEXT_START,
};
typedef Flags<InterfaceFlag> InterfaceFlags;

// some useful constants
constexpr InterfaceFlags UNKNOWN;
constexpr InterfaceFlags START_IF_MASK({ READS_START, WRITES_PREV_END });
constexpr InterfaceFlags END_IF_MASK({ READS_END, WRITES_NEXT_START });
constexpr InterfaceFlags PROPAGATE_BOTHWAYS({ PROPAGATE_FORWARDS, PROPAGATE_BACKWARDS });

MOVEIT_CLASS_FORWARD(Interface)
MOVEIT_CLASS_FORWARD(Stage)
class InterfaceState;
typedef std::pair<const InterfaceState&, const InterfaceState&> InterfaceStatePair;

/// exception thrown by Stage::init()
/// It collects individual errors in stages throughout the pipeline to allow overall error reporting
class InitStageException : public std::exception
{
	friend std::ostream& operator<<(std::ostream& os, const InitStageException& e);

public:
	explicit InitStageException() {}
	explicit InitStageException(const Stage& stage, const std::string& msg) { push_back(stage, msg); }

	/// push_back a single new error in stage
	void push_back(const Stage& stage, const std::string& msg);
	/// append all the errors from other (which is emptied)
	void append(InitStageException& other);

	/// check of existing errors
	operator bool() const { return !errors_.empty(); }

	virtual const char* what() const noexcept override;

private:
	std::list<std::pair<const Stage*, const std::string>> errors_;
};
std::ostream& operator<<(std::ostream& os, const InitStageException& e);

class ContainerBase;
class StagePrivate;
class Stage
{
public:
	PRIVATE_CLASS(Stage)
	typedef std::unique_ptr<Stage> pointer;
	/** Names for property initialization sources
	 *
	 * - INTERFACE allows to pass properties from one stage to the next (in a SerialContainer).
	 * - PARENT allows to inherit properties from the parent.
	 *
	 * INTERFACE takes precedence over PARENT.
	 */
	enum PropertyInitializerSource
	{  // TODO: move to property.cpp
		DEFAULT = 0,
		MANUAL = 1,
		PARENT = 2,
		INTERFACE = 4,
	};
	virtual ~Stage();

	/// auto-convert Stage to StagePrivate* when needed
	operator StagePrivate*();
	operator const StagePrivate*() const;

	/// reset stage, clearing all solutions, interfaces, inherited properties.
	virtual void reset();

	/** initialize stage once before planning
	 *
	 * When called, properties configured for initialization from parent are already defined.
	 * Push interfaces are not yet defined!
	 */
	virtual void init(const moveit::core::RobotModelConstPtr& robot_model);

	const ContainerBase* parent() const;

	const std::string& name() const;
	void setName(const std::string& name);

	/** set computation timeout (in seconds)
	 *
	 * The logic of the individual stage should ensure this limit is respected.
	 */
	void setTimeout(double timeout) { setProperty("timeout", timeout); }
	/// timeout of stage per computation
	double timeout() const { return properties().get<double>("timeout"); }

	/** set marker namespace for solutions
	 *
	 * Auxiliary markers in this stage should use this namespace
	 */
	void setMarkerNS(const std::string& marker_ns) { setProperty("marker_ns", marker_ns); }
	/// marker namespace of solution markers
	const std::string& markerNS() { return properties().get<std::string>("marker_ns"); }

	/// forwarding of properties between interface states
	void forwardProperties(const InterfaceState& source, InterfaceState& dest);
	std::set<std::string> forwardedProperties() const {
		return properties().get<std::set<std::string>>("forwarded_properties");
	}
	void setForwardedProperties(const std::set<std::string>& names) { setProperty("forwarded_properties", names); }

	typedef std::function<void(const SolutionBase& s)> SolutionCallback;
	typedef std::list<SolutionCallback> SolutionCallbackList;
	/// add function to be called for every newly found solution or failure
	SolutionCallbackList::const_iterator addSolutionCallback(SolutionCallback&& cb);
	/// remove function callback
	void removeSolutionCallback(SolutionCallbackList::const_iterator which);

	const ordered<SolutionBaseConstPtr>& solutions() const;
	const std::list<SolutionBaseConstPtr>& failures() const;
	size_t numFailures() const;
	/// call to increase number of failures w/o storing a (failure) trajectory
	void silentFailure();
	/// should we generate failure solutions?
	bool storeFailures() const;

	/// get the stage's property map
	PropertyMap& properties();
	const PropertyMap& properties() const { return const_cast<Stage*>(this)->properties(); }
	/// set a previously declared property to a new value
	void setProperty(const std::string& name, const boost::any& value);
	/// overload: const char* values are stored as std::string
	inline void setProperty(const std::string& name, const char* value) { setProperty(name, std::string(value)); }
	/// analyze source of error and report accordingly
	void reportPropertyError(const Property::error& e);

protected:
	/// Stage can only be instantiated through derived classes
	Stage(StagePrivate* impl);
	/// Stage cannot be copied
	Stage(const Stage&) = delete;

protected:
	StagePrivate* pimpl_;
};
std::ostream& operator<<(std::ostream& os, const Stage& stage);

class ComputeBasePrivate;
class ComputeBase : public Stage
{
public:
	PRIVATE_CLASS(ComputeBase)

protected:
	/// ComputeBase can only be instantiated by derived classes in stage.cpp
	ComputeBase(ComputeBasePrivate* impl);
};

class PropagatingEitherWayPrivate;
/** Base class for stages that can propagate InterfaceStates
 *
 *  They read an InterfaceState on one side (start or end) and
 *  push a new InterfaceState to the opposite site.
 *  By default, the class auto-derives its actual propagation direction from the context.
 *  In order to enforce forward, backward, or bothway propagation, one can use restrictDirection().
 */
class PropagatingEitherWay : public ComputeBase
{
public:
	PRIVATE_CLASS(PropagatingEitherWay)
	PropagatingEitherWay(const std::string& name = "propagating either way");

	enum Direction
	{
		AUTO = 0x00,  // auto-derive direction from context
		FORWARD = 0x01,  // propagate forward only
		BACKWARD = 0x02,  // propagate backward only
		BOTHWAY = FORWARD | BACKWARD,  // propagate both ways
	};
	void restrictDirection(Direction dir);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

	virtual void computeForward(const InterfaceState& from) = 0;
	virtual void computeBackward(const InterfaceState& to) = 0;

	void sendForward(const InterfaceState& from, InterfaceState&& to, SubTrajectory&& trajectory);
	void sendBackward(InterfaceState&& from, const InterfaceState& to, SubTrajectory&& trajectory);

protected:
	// constructor for use in derived classes
	PropagatingEitherWay(PropagatingEitherWayPrivate* impl);
};

class PropagatingForwardPrivate;
class PropagatingForward : public PropagatingEitherWay
{
public:
	PRIVATE_CLASS(PropagatingForward)
	PropagatingForward(const std::string& name = "propagating forward");

private:
	// restrict access to backward method to provide compile-time check
	void computeBackward(const InterfaceState& to) override;
	using PropagatingEitherWay::sendBackward;
};

class PropagatingBackwardPrivate;
class PropagatingBackward : public PropagatingEitherWay
{
public:
	PRIVATE_CLASS(PropagatingBackward)
	PropagatingBackward(const std::string& name = "propagating backward");

private:
	// restrict access to forward method to provide compile-time check
	void computeForward(const InterfaceState& from) override;
	using PropagatingEitherWay::sendForward;
};

class GeneratorPrivate;
class Generator : public ComputeBase
{
public:
	PRIVATE_CLASS(Generator)
	Generator(const std::string& name = "generator");

	virtual bool canCompute() const = 0;
	virtual void compute() = 0;
	void spawn(InterfaceState&& state, SubTrajectory&& trajectory);
	void spawn(InterfaceState&& state, double cost) {
		SubTrajectory trajectory;
		trajectory.setCost(cost);
		spawn(std::move(state), std::move(trajectory));
	}

protected:
	Generator(GeneratorPrivate* impl);
};

class MonitoringGeneratorPrivate;
/** Generator that monitors solutions of another stage to make reuse of them
 *
 * Sometimes its necessary to reuse a previously planned solution, e.g. to traverse
 * it in reverse order or to access the state of another generator.
 * To this end, the present stage hooks into the onNewSolution() method of the
 * monitored stage and forwards it to this' class onNewSolution() method.
 */
class MonitoringGenerator : public Generator
{
public:
	PRIVATE_CLASS(MonitoringGenerator)
	MonitoringGenerator(const std::string& name = "monitoring generator", Stage* monitored = nullptr);
	void setMonitoredStage(Stage* monitored);

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;

protected:
	MonitoringGenerator(MonitoringGeneratorPrivate* impl);

	/// called by monitored stage when a new solution was generated
	virtual void onNewSolution(const SolutionBase& s) = 0;
};

class ConnectingPrivate;
class Connecting : public ComputeBase
{
protected:
	virtual bool compatible(const InterfaceState& from_state, const InterfaceState& to_state) const;

public:
	PRIVATE_CLASS(Connecting)
	Connecting(const std::string& name = "connecting");

	void reset() override;

	virtual void compute(const InterfaceState& from, const InterfaceState& to) = 0;

protected:
	/// register solution as a solution connecting states from -> to
	void connect(const InterfaceState& from, const InterfaceState& to, SolutionBasePtr solution);

	/// convienency methods consuming a SubTrajectory
	void connect(const InterfaceState& from, const InterfaceState& to, SubTrajectory&& trajectory) {
		connect(from, to, std::make_shared<SubTrajectory>(std::move(trajectory)));
	}
	void connect(const InterfaceState& from, const InterfaceState& to, SubTrajectory&& trajectory, double cost) {
		trajectory.setCost(cost);
		connect(from, to, std::move(trajectory));
	}
};

const char* flowSymbol(moveit::task_constructor::InterfaceFlags f);
}
}
