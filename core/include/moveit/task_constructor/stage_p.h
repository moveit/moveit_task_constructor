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

// define pimpl() functions accessing correctly casted pimpl_ pointer
#define PIMPL_FUNCTIONS(Class) \
	const Class##Private* Class::pimpl() const { return static_cast<const Class##Private*>(pimpl_); } \
	Class##Private* Class::pimpl() { return static_cast<Class##Private*>(pimpl_); } \

namespace moveit { namespace task_constructor {

class ContainerBase;
class StagePrivate {
	friend class Stage;
	friend std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);

public:
	typedef std::list<Stage::pointer> container_type;
	StagePrivate(Stage* me, const std::string& name);
	virtual ~StagePrivate() = default;

	InterfaceFlags interfaceFlags() const;

	virtual bool canCompute() const = 0;
	virtual bool compute() = 0;

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

	/// the following methods should be called only by a container
	/// to setup the connection structure of their children
	inline void setHierarchy(ContainerBase* parent, container_type::iterator it) {
		parent_ = parent;
		it_ = it;
	}
	inline void setPrevEnds(const InterfacePtr& prev_ends) { prev_ends_ = prev_ends; }
	inline void setNextStarts(const InterfacePtr& next_starts) { next_starts_ = next_starts; }
	inline void setIntrospection(Introspection* introspection) { introspection_ = introspection; }

	void newSolution(const SolutionBase &current);

protected:
	Stage* const me_; // associated/owning Stage instance
	std::string name_;
	PropertyMap properties_;

	InterfacePtr starts_;
	InterfacePtr ends_;

	// functions called for each new solution
	std::list<Stage::SolutionCallback> solution_cbs_;

private:
	// !! items write-accessed only by ContainerBasePrivate to maintain hierarchy !!
	ContainerBase* parent_;       // owning parent
	container_type::iterator it_; // iterator into parent's children_ list referring to this

	InterfaceWeakPtr prev_ends_;    // interface to be used for sendBackward()
	InterfaceWeakPtr next_starts_;  // interface to be used for sendForward()

	Introspection* introspection_;  // task's introspection instance
};
PIMPL_FUNCTIONS(Stage)
std::ostream& operator<<(std::ostream &os, const StagePrivate& stage);


// ComputeBasePrivate is the base class for all computing stages, i.e. non-containers.
// It adds the trajectories_ variable.
class ComputeBasePrivate : public StagePrivate {
	friend class ComputeBase;

public:
	ComputeBasePrivate(Stage* me, const std::string& name)
	   : StagePrivate(me, name)
	{}

	// store trajectory in internal trajectories_ list
	SubTrajectory& addTrajectory(SubTrajectory&& trajectory);

	// countFailures() serves as a filter before returning the result of compute()
	inline bool countFailures(bool success) {
		if (!success) ++num_failures_;
		return success;
	}

private:
	ordered<SubTrajectory> solutions_;
	std::list<SubTrajectory> failures_;
	size_t num_failures_ = 0;  // num of failures if not stored
};
PIMPL_FUNCTIONS(ComputeBase)


class PropagatingEitherWayPrivate : public ComputeBasePrivate {
	friend class PropagatingEitherWay;

public:
	PropagatingEitherWay::Direction dir;

	inline PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir,
	                                   const std::string &name);

	// returns true if prevEnds() or nextStarts() are accessible
	inline bool isConnected() const { return prevEnds() || nextStarts(); }

	bool canCompute() const override;
	bool compute() override;

	bool hasStartState() const;
	const InterfaceState &fetchStartState();

	bool hasEndState() const;
	const InterfaceState &fetchEndState();

protected:
	// get informed when new start or end state becomes available
	void newStartState(const std::list<InterfaceState>::iterator& it);
	void newEndState(const std::list<InterfaceState>::iterator& it);
	// initialize properties from parent and/or state
	void initProperties(const InterfaceState &state);

	Interface::const_iterator next_start_state_;
	Interface::const_iterator next_end_state_;
};
PIMPL_FUNCTIONS(PropagatingEitherWay)


class PropagatingForwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingForwardPrivate(PropagatingForward *me, const std::string &name);
};
PIMPL_FUNCTIONS(PropagatingForward)


class PropagatingBackwardPrivate : public PropagatingEitherWayPrivate {
public:
	inline PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name);
};
PIMPL_FUNCTIONS(PropagatingBackward)


class GeneratorPrivate : public ComputeBasePrivate {
public:
	inline GeneratorPrivate(Generator *me, const std::string &name);

	bool canCompute() const override;
	bool compute() override;

private:
	// initialize properties from parent
	void initProperties();
};
PIMPL_FUNCTIONS(Generator)


class ConnectingPrivate : public ComputeBasePrivate {
	friend class Connecting;

public:
	inline ConnectingPrivate(Connecting *me, const std::string &name);

	bool canCompute() const override;
	bool compute() override;

	void connect(const robot_trajectory::RobotTrajectoryPtr& t,
	             const InterfaceStatePair& state_pair, double cost);

private:
	// get informed when new start or end state becomes available
	void newStartState(const std::list<InterfaceState>::iterator& it);
	void newEndState(const std::list<InterfaceState>::iterator& it);
	// initialize properties from parent and/or interface states
	void initProperties(const InterfaceState &start, const InterfaceState &end);

	std::pair<Interface::const_iterator, Interface::const_iterator> it_pairs_;
};
PIMPL_FUNCTIONS(Connecting)

} }
