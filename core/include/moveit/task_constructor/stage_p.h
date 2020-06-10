/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Authors: Robert Haschke
   Desc:   Private Implementation for the Stage classes
*/

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/cost_queue.h>

#include <ros/ros.h>

#include <ostream>
#include <chrono>

// define pimpl() functions accessing correctly casted pimpl_ pointer
#define PIMPL_FUNCTIONS(Class)                                                                       \
	const Class##Private* Class::pimpl() const { return static_cast<const Class##Private*>(pimpl_); } \
	Class##Private* Class::pimpl() { return static_cast<Class##Private*>(pimpl_); }

namespace moveit {
namespace task_constructor {

class ContainerBase;
class StagePrivate
{
	friend class Stage;
	friend std::ostream& operator<<(std::ostream& os, const StagePrivate& stage);
	friend void swap(StagePrivate*& lhs, StagePrivate*& rhs);

public:
	/// container type used to store children
	typedef std::list<Stage::pointer> container_type;
	StagePrivate(Stage* me, const std::string& name);
	virtual ~StagePrivate() = default;

	/// actually configured interface of this stage (only valid after init())
	InterfaceFlags interfaceFlags() const;

	/// Retrieve interface required by this stage, UNKNOWN if auto-detected from context
	virtual InterfaceFlags requiredInterface() const = 0;

	/// Resolve interface/propagation direction to comply with the given external interface
	virtual void resolveInterface(InterfaceFlags /* expected */) {}

	/// validate connectivity of children (after init() was done)
	virtual void validateConnectivity() const;

	virtual bool canCompute() const = 0;
	virtual void compute() = 0;

	inline const Stage* me() const { return me_; }
	inline Stage* me() { return me_; }
	inline const std::string& name() const { return name_; }
	inline const ContainerBase* parent() const { return parent_; }
	inline ContainerBase* parent() { return parent_; }
	inline container_type::const_iterator it() const { return it_; }

	inline InterfacePtr& starts() { return starts_; }
	inline InterfacePtr& ends() { return ends_; }
	inline InterfacePtr prevEnds() { return prev_ends_.lock(); }
	inline InterfacePtr nextStarts() { return next_starts_.lock(); }
	inline InterfaceConstPtr starts() const { return starts_; }
	inline InterfaceConstPtr ends() const { return ends_; }
	inline InterfaceConstPtr prevEnds() const { return prev_ends_.lock(); }
	inline InterfaceConstPtr nextStarts() const { return next_starts_.lock(); }

	/// templated access to pull/push interfaces
	inline InterfacePtr& pullInterface(Interface::Direction dir) { return dir == Interface::FORWARD ? starts_ : ends_; }
	inline InterfacePtr pushInterface(Interface::Direction dir) {
		return dir == Interface::FORWARD ? next_starts_.lock() : prev_ends_.lock();
	}
	inline InterfaceConstPtr pullInterface(Interface::Direction dir) const {
		return dir == Interface::FORWARD ? starts_ : ends_;
	}
	inline InterfaceConstPtr pushInterface(Interface::Direction dir) const {
		return dir == Interface::FORWARD ? next_starts_.lock() : prev_ends_.lock();
	}

	/// set parent of stage
	/// enforce only one parent exists
	inline bool setParent(ContainerBase* parent) {
		if (parent_) {
			ROS_ERROR_STREAM("Tried to add stage '" << name() << "' to two parents");
			return false;  // it's not allowed to add a stage to a parent if it already has one
		}
		parent_ = parent;
		return true;
	}

	/// explicitly orphan stage
	inline void unparent() {
		parent_ = nullptr;
		it_ = container_type::iterator();
	}

	/// the following methods should be called only by the current parent
	/// to setup the connection structure of their children
	inline void setParentPosition(container_type::iterator it) { it_ = it; }
	inline void setIntrospection(Introspection* introspection) { introspection_ = introspection; }

	inline void setPrevEnds(const InterfacePtr& prev_ends) { prev_ends_ = prev_ends; }
	inline void setNextStarts(const InterfacePtr& next_starts) { next_starts_ = next_starts; }

	void composePropertyErrorMsg(const std::string& name, std::ostream& os);

	// methods to spawn new solutions
	void sendForward(const InterfaceState& from, InterfaceState&& to, const SolutionBasePtr& solution);
	void sendBackward(InterfaceState&& from, const InterfaceState& to, const SolutionBasePtr& solution);
	void spawn(InterfaceState&& state, const SolutionBasePtr& solution);
	void connect(const InterfaceState& from, const InterfaceState& to, const SolutionBasePtr& solution);

	bool storeSolution(const SolutionBasePtr& solution);
	void newSolution(const SolutionBasePtr& solution);
	bool storeFailures() const { return introspection_ != nullptr; }
	void runCompute() {
		auto compute_start_time = std::chrono::steady_clock::now();
		compute();
		auto compute_stop_time = std::chrono::steady_clock::now();
		total_compute_time_ += compute_stop_time - compute_start_time;
	}

protected:
	Stage* me_;  // associated/owning Stage instance
	std::string name_;
	PropertyMap properties_;

	// pull interfaces, created by the stage as required
	InterfacePtr starts_;
	InterfacePtr ends_;

	// The total compute time
	std::chrono::duration<double> total_compute_time_;

	// functions called for each new solution
	std::list<Stage::SolutionCallback> solution_cbs_;

	std::list<InterfaceState> states_;  // storage for created states
	ordered<SolutionBaseConstPtr> solutions_;
	std::list<SolutionBaseConstPtr> failures_;
	size_t num_failures_ = 0;  // num of failures if not stored

private:
	// !! items write-accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBase* parent_;  // owning parent
	container_type::iterator it_;  // iterator into parent's children_ list referring to this

	// push interfaces, assigned by the parent container
	// linking to previous/next sibling's pull interfaces
	InterfaceWeakPtr prev_ends_;  // interface to be used for sendBackward()
	InterfaceWeakPtr next_starts_;  // interface to be used for sendForward()

	Introspection* introspection_;  // task's introspection instance
};
PIMPL_FUNCTIONS(Stage)
std::ostream& operator<<(std::ostream& os, const StagePrivate& stage);

// ComputeBasePrivate is the base class for all computing stages, i.e. non-containers.
// It adds the trajectories_ variable.
class ComputeBasePrivate : public StagePrivate
{
	friend class ComputeBase;

public:
	ComputeBasePrivate(Stage* me, const std::string& name) : StagePrivate(me, name) {}

private:
};
PIMPL_FUNCTIONS(ComputeBase)

class PropagatingEitherWayPrivate : public ComputeBasePrivate
{
	friend class PropagatingEitherWay;

public:
	PropagatingEitherWay::Direction configured_dir_;
	InterfaceFlags required_interface_;

	inline PropagatingEitherWayPrivate(PropagatingEitherWay* me, PropagatingEitherWay::Direction configured_dir_,
	                                   const std::string& name);

	InterfaceFlags requiredInterface() const override;
	// initialize pull interfaces for given propagation directions
	void initInterface(PropagatingEitherWay::Direction dir);
	// resolve interface to the given propagation direction
	void resolveInterface(InterfaceFlags expected) override;

	bool canCompute() const override;
	void compute() override;

	bool hasStartState() const;
	const InterfaceState& fetchStartState();

	bool hasEndState() const;
	const InterfaceState& fetchEndState();

protected:
	// drop states corresponding to failed (infinite-cost) trajectories
	void dropFailedStarts(Interface::iterator state);
	void dropFailedEnds(Interface::iterator state);
};
PIMPL_FUNCTIONS(PropagatingEitherWay)

class PropagatingForwardPrivate : public PropagatingEitherWayPrivate
{
public:
	inline PropagatingForwardPrivate(PropagatingForward* me, const std::string& name);
};
PIMPL_FUNCTIONS(PropagatingForward)

class PropagatingBackwardPrivate : public PropagatingEitherWayPrivate
{
public:
	inline PropagatingBackwardPrivate(PropagatingBackward* me, const std::string& name);
};
PIMPL_FUNCTIONS(PropagatingBackward)

class GeneratorPrivate : public ComputeBasePrivate
{
public:
	inline GeneratorPrivate(Generator* me, const std::string& name);

	InterfaceFlags requiredInterface() const override;
	bool canCompute() const override;
	void compute() override;
};
PIMPL_FUNCTIONS(Generator)

class MonitoringGeneratorPrivate : public GeneratorPrivate
{
	friend class MonitoringGenerator;

public:
	Stage* monitored_;
	Stage::SolutionCallbackList::const_iterator cb_;
	bool registered_;

	inline MonitoringGeneratorPrivate(MonitoringGenerator* me, const std::string& name);

private:
	void solutionCB(const SolutionBase& s);
};
PIMPL_FUNCTIONS(MonitoringGenerator)

class ConnectingPrivate : public ComputeBasePrivate
{
	friend class Connecting;

	using StatePair = std::pair<Interface::const_iterator, Interface::const_iterator>;
	struct StatePairLess
	{
		bool operator()(const StatePair& x, const StatePair& y) const {
			return x.first->priority() + x.second->priority() < y.first->priority() + y.second->priority();
		}
	};

	template <Interface::Direction other>
	inline StatePair make_pair(Interface::const_iterator first, Interface::const_iterator second);

public:
	inline ConnectingPrivate(Connecting* me, const std::string& name);

	InterfaceFlags requiredInterface() const override;
	bool canCompute() const override;
	void compute() override;

private:
	// get informed when new start or end state becomes available
	template <Interface::Direction other>
	void newState(Interface::iterator it, bool updated);

	// ordered list of pending state pairs
	ordered<StatePair, StatePairLess> pending;
};
PIMPL_FUNCTIONS(Connecting)
}  // namespace task_constructor
}  // namespace moveit
