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
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/cost_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <fmt/core.h>

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

public:
	/// container type used to store children
	using container_type = std::list<Stage::pointer>;
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

	/// direction-based access to pull interface
	template <Interface::Direction dir>
	inline InterfacePtr pullInterface();
	// non-template version
	inline InterfacePtr pullInterface(Interface::Direction dir) { return dir == Interface::FORWARD ? starts_ : ends_; }

	/// set parent of stage
	/// enforce only one parent exists
	inline void setParent(ContainerBase* parent) {
		if (parent_) {
			// it's not allowed to add a stage to a parent if it already has one
			throw std::runtime_error("Tried to add stage '" + name() + "' to two parents");
		}
		parent_ = parent;
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
	template <Interface::Direction>
	inline void send(const InterfaceState& start, InterfaceState&& end, const SolutionBasePtr& solution);
	void spawn(InterfaceState&& from, InterfaceState&& to, const SolutionBasePtr& solution);
	void spawn(InterfaceState&& state, const SolutionBasePtr& solution);
	void connect(const InterfaceState& from, const InterfaceState& to, const SolutionBasePtr& solution);

	bool storeSolution(const SolutionBasePtr& solution, const InterfaceState* from, const InterfaceState* to);
	void newSolution(const SolutionBasePtr& solution);
	bool storeFailures() const { return introspection_ != nullptr; }
	void runCompute() {
		RCLCPP_DEBUG_STREAM(LOGGER, fmt::format("Computing stage '{}'", name()));
		auto compute_start_time = std::chrono::steady_clock::now();
		try {
			compute();
		} catch (const Property::error& e) {
			me()->reportPropertyError(e);
		}
		auto compute_stop_time = std::chrono::steady_clock::now();
		total_compute_time_ += compute_stop_time - compute_start_time;
	}

	/** compute cost for solution through configured CostTerm */
	void computeCost(const InterfaceState& from, const InterfaceState& to, SolutionBase& solution);

protected:
	StagePrivate& operator=(StagePrivate&& other);

	// associated/owning Stage instance
	Stage* me_;

	std::string name_;
	PropertyMap properties_;

	// pull interfaces, created by the stage as required
	InterfacePtr starts_;
	InterfacePtr ends_;

	// user-configurable cost estimator
	CostTermConstPtr cost_term_;

	// The total compute time
	std::chrono::duration<double> total_compute_time_;

	// functions called for each new solution
	std::list<Stage::SolutionCallback> solution_cbs_;

	std::list<InterfaceState> states_;  // storage for created states
	ordered<SolutionBaseConstPtr> solutions_;
	std::list<SolutionBaseConstPtr> failures_;
	std::size_t num_failures_ = 0;  // num of failures if not stored

private:
	// !! items write-accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBase* parent_;  // owning parent
	container_type::iterator it_;  // iterator into parent's children_ list referring to this

	// push interfaces, assigned by the parent container
	// linking to previous/next sibling's pull interfaces
	InterfaceWeakPtr prev_ends_;  // interface to be used for sendBackward()
	InterfaceWeakPtr next_starts_;  // interface to be used for sendForward()

	Introspection* introspection_;  // task's introspection instance
	inline static const rclcpp::Logger LOGGER = rclcpp::get_logger("stage");
};
PIMPL_FUNCTIONS(Stage)
std::ostream& operator<<(std::ostream& os, const StagePrivate& stage);

template <>
inline InterfacePtr StagePrivate::pullInterface<Interface::FORWARD>() {
	return starts_;
}
template <>
inline InterfacePtr StagePrivate::pullInterface<Interface::BACKWARD>() {
	return ends_;
}

template <>
inline void StagePrivate::send<Interface::FORWARD>(const InterfaceState& start, InterfaceState&& end,
                                                   const SolutionBasePtr& solution) {
	sendForward(start, std::move(end), solution);
}
template <>
inline void StagePrivate::send<Interface::BACKWARD>(const InterfaceState& start, InterfaceState&& end,
                                                    const SolutionBasePtr& solution) {
	sendBackward(std::move(end), start, solution);
}

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

// Print pending pairs of a ConnectingPrivate instance
class ConnectingPrivate;
struct PendingPairsPrinter
{
	const ConnectingPrivate* const instance_;
	PendingPairsPrinter(const ConnectingPrivate* c) : instance_(c) {}
};
std::ostream& operator<<(std::ostream& os, const PendingPairsPrinter& printer);

class ConnectingPrivate : public ComputeBasePrivate
{
	friend class Connecting;
	friend struct FallbacksPrivateConnect;
	friend std::ostream& operator<<(std::ostream& os, const PendingPairsPrinter& printer);

public:
	struct StatePair : std::pair<Interface::const_iterator, Interface::const_iterator>
	{
		using std::pair<Interface::const_iterator, Interface::const_iterator>::pair;  // inherit base constructors
		bool operator<(const StatePair& rhs) const {
			return less(first->priority(), second->priority(), rhs.first->priority(), rhs.second->priority());
		}
		static inline bool less(const InterfaceState::Priority& lhsA, const InterfaceState::Priority& lhsB,
		                        const InterfaceState::Priority& rhsA, const InterfaceState::Priority& rhsB) {
			bool lhs = lhsA.enabled() && lhsB.enabled();
			bool rhs = rhsA.enabled() && rhsB.enabled();

			if (lhs == rhs)  // if enabled status is identical
				return lhsA + lhsB < rhsA + rhsB;  // compare the sums of both contributions

			// sort both-enabled pairs first
			static_assert(true > false, "Comparing enabled states requires true > false");
			return lhs > rhs;
		}
	};

	inline ConnectingPrivate(Connecting* me, const std::string& name);

	InterfaceFlags requiredInterface() const override;
	bool canCompute() const override;
	void compute() override;

	// Check whether there are pending feasible states that could connect to source
	template <Interface::Direction dir>
	bool hasPendingOpposites(const InterfaceState* source, const InterfaceState* target) const;

	PendingPairsPrinter pendingPairsPrinter() const { return PendingPairsPrinter(this); }

private:
	// Create a pair of Interface states for pending list, such that the order (start, end) is maintained
	template <Interface::Direction other>
	inline StatePair make_pair(Interface::const_iterator first, Interface::const_iterator second);

	// notify callback to get informed about newly inserted (or updated) start or end states
	template <Interface::Direction other>
	void newState(Interface::iterator it, Interface::UpdateFlags updated);

	// ordered list of pending state pairs
	ordered<StatePair> pending;
};
PIMPL_FUNCTIONS(Connecting)

}  // namespace task_constructor
}  // namespace moveit
