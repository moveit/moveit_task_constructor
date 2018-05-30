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

/* Author: Robert Haschke
   Desc:   Private Implementation of the Containers
*/

#pragma once

#include <moveit/task_constructor/container.h>
#include <moveit/macros/class_forward.h>
#include "stage_p.h"

#include <map>
#include <climits>

namespace moveit { namespace core {
MOVEIT_CLASS_FORWARD(JointModelGroup)
MOVEIT_CLASS_FORWARD(RobotState)
} }

namespace moveit { namespace task_constructor {

/* A container needs to decouple its own (external) interfaces
 * from those (internal) of its children.
 * Both, the container and the children have their own pull interfaces: starts_ and ends_.
 * The container forwards states received in its pull interfaces to the
 * corresponding interfaces of the children.
 * States pushed by children are temporarily stored in pending_backward_ and pending_forward_.
 *
 * Solutions found by the children need to be lifted from the internal level to the
 * external level. To this end, we remember the mapping from internal to external states.
 *
 * Note, that there might be many solutions connecting a single start-end pair.
 * These solutions might origin from different children (ParallelContainer)
 * or from different solution paths in a SerialContainer.
 */
class ContainerBasePrivate : public StagePrivate
{
	friend class ContainerBase;

public:
	typedef StagePrivate::container_type container_type;
	typedef container_type::iterator iterator;
	typedef container_type::const_iterator const_iterator;
	typedef std::function<bool(Stage&, int depth)> NonConstStageCallback;

	inline const container_type& children() const { return children_; }

	/** Retrieve iterator into children_ pointing to indexed element.
	 * Negative index counts from end, i.e. -1 is last, -2 is second last, etc. (if !for_insert)
	 * If for_insert == true, negative indexes are shifted by one: -1 is end(), -2 is --end(), etc.
	 * Contrary to std::advance(), iterator limits are considered. */
	const_iterator childByIndex(int index, bool for_insert = false) const;

	/// traversing all stages up to max_depth
	bool traverseStages(const ContainerBase::StageCallback &processor,
	                    unsigned int cur_depth, unsigned int max_depth) const;

	/// non-const version
	bool traverseStages(const NonConstStageCallback &processor,
	                    unsigned int cur_depth, unsigned int max_depth) {
		const auto& const_processor = [&processor](const Stage& stage, unsigned int depth) {
			return processor(const_cast<Stage&>(stage), depth);
		};
		return const_cast<const ContainerBasePrivate*>(this)->traverseStages(const_processor, cur_depth, max_depth);
	}

	// forward these methods to the public interface for containers
	bool canCompute() const override;
	bool compute() override;

	InterfacePtr pendingBackward() const { return pending_backward_; }
	InterfacePtr pendingForward() const { return pending_forward_; }

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name);

	// Set child's push interfaces: allow pushing if child requires it or
	// if the interface is unknown: in this case greedily assume a push interface.
	// If, during pruneInterface() later, we notice that it's not needed, prune there.
	inline void setChildsPushBackwardInterface(Stage& child) {
		InterfaceFlags required = child.pimpl()->requiredInterface();
		bool allowed = (required & WRITES_PREV_END) || (required == UNKNOWN);
		child.pimpl()->setPrevEnds(allowed ? pending_backward_ : InterfacePtr());
	}
	inline void setChildsPushForwardInterface(Stage& child) {
		InterfaceFlags required = child.pimpl()->requiredInterface();
		bool allowed = (required & WRITES_NEXT_START) || (required == UNKNOWN);
		child.pimpl()->setNextStarts(allowed ? pending_forward_ : InterfacePtr());
	}

	/// copy external_state to a child's interface and remember the link in internal_to map
	void copyState(Interface::iterator external, const InterfacePtr& target, bool updated);
	/// lift solution from internal to external level
	void liftSolution(SolutionBase& solution,
	                  const InterfaceState *internal_from, const InterfaceState *internal_to);

	auto& internalToExternalMap() { return internal_to_external_; }
	const auto& internalToExternalMap() const { return internal_to_external_; }

private:
	container_type children_;

	// map start/end states of children (internal) to corresponding states in our external interfaces
	std::map<const InterfaceState*, Interface::iterator> internal_to_external_;

	/* TODO: these interfaces don't need to be priority-sorted.
	 * Introduce base class UnsortedInterface (which is a plain list) for this use case. */
	// interface to receive children's sendBackward() states
	InterfacePtr pending_backward_;
	// interface to receive children's sendForward() states
	InterfacePtr pending_forward_;
};
PIMPL_FUNCTIONS(ContainerBase)


/* A solution of a SerialContainer needs to connect start to end via a full path.
 * The solution of a single child stage is usually disconnected to the container's start or end.
 * Only if all the children in the chain have found a coherent solution from start to end,
 * this solution can be announced as a solution of the SerialContainer.
 */
class SerialContainerPrivate : public ContainerBasePrivate {
	friend class SerialContainer;

public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name);

	// containers derive their required interface from their children
	InterfaceFlags requiredInterface() const override;

	// called by parent asking for pruning of this' interface
	void pruneInterface(InterfaceFlags accepted) override;

private:
	// connect cur stage to its predecessor and successor
	bool connect(container_type::const_iterator cur);

	// called by init() to prune interfaces for children in range [first, last)
	void pruneInterfaces(container_type::const_iterator first,
	                     container_type::const_iterator end);
	// prune interface for children in range [first, last) to given direction
	void pruneInterfaces(container_type::const_iterator first,
	                     container_type::const_iterator end,
	                     InterfaceFlags accepted);

	// set of all solutions
	ordered<SolutionSequence> solutions_;
};
PIMPL_FUNCTIONS(SerialContainer)


/** Wrap an existing solution - for use in parallel containers and wrappers.
 *
 * This essentially wraps a solution of a child and thus allows
 * for new clones of start / end states, which in turn will
 * have separate incoming/outgoing trajectories */
class WrappedSolution : public SolutionBase {
public:
	explicit WrappedSolution(StagePrivate* creator, const SolutionBase* wrapped, double cost)
		: SolutionBase(creator, cost), wrapped_(wrapped)
	{}
	explicit WrappedSolution(StagePrivate* creator, const SolutionBase* wrapped)
		: WrappedSolution(creator, wrapped, wrapped->cost())
	{}
	void fillMessage(moveit_task_constructor_msgs::Solution &solution,
	                 Introspection* introspection = nullptr) const override;

private:
	const SolutionBase* wrapped_;
};


class ParallelContainerBasePrivate : public ContainerBasePrivate {
	friend class ParallelContainerBase;

public:
	ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string &name);

	// containers derive their required interface from their children
	InterfaceFlags requiredInterface() const override;

	// called by parent asking for pruning of this' interface
	void pruneInterface(InterfaceFlags accepted) override;

	// grant access to protected methods in ParallelContainerBase
	inline void spawn(InterfaceState &&state, SubTrajectory&& t) {
		static_cast<ParallelContainerBase*>(me_)->spawn(std::move(state), std::move(t));
	}
	inline void sendForward(const InterfaceState& from, InterfaceState&& to, SubTrajectory&& t) {
		static_cast<ParallelContainerBase*>(me_)->sendForward(from, std::move(to), std::move(t));
	}
	inline void sendBackward(InterfaceState&& from, const InterfaceState& to, SubTrajectory&& t) {
		static_cast<ParallelContainerBase*>(me_)->sendBackward(std::move(from), to, std::move(t));
	}

private:
	/// callback for new externally received states
	void onNewExternalState(Interface::Direction dir, Interface::iterator external, bool updated);

	// buffer for wrapped solutions
	std::list<WrappedSolution> wrapped_solutions_;
	// buffer for newly created (not wrapped) solutions
	std::list<SubTrajectory> created_solutions_;
	// buffer of created states (for use in created solutions)
	std::list<InterfaceState> states_;

	// cost-ordered set of solutions (pointers into wrapped or created)
	ordered<SolutionBase*> solutions_;
	// buffer for failures (pointers into wrapped or created)
	std::list<SolutionBase*> failures_;
};
PIMPL_FUNCTIONS(ParallelContainerBase)


class WrapperBasePrivate : public ParallelContainerBasePrivate {
	friend class WrapperBase;

public:
	WrapperBasePrivate(WrapperBase* me, const std::string& name);
};
PIMPL_FUNCTIONS(WrapperBase)


class MergerPrivate : public ParallelContainerBasePrivate {
	friend class Merger;

	moveit::core::JointModelGroupPtr jmg_merged_;
	typedef std::vector<const SubTrajectory*> ChildSolutionList;
	typedef std::map<const StagePrivate*, ChildSolutionList> ChildSolutionMap;
	// map from external source state (iterator) to all corresponding children's solutions
	std::map<InterfaceState*, ChildSolutionMap> source_state_to_solutions_;

public:
	typedef std::function<void(SubTrajectory&&)> Spawner;
	MergerPrivate(Merger* me, const std::string& name);

	InterfaceFlags requiredInterface() const override;

	void onNewPropagateSolution(const SolutionBase& s);
	void onNewGeneratorSolution(const SolutionBase& s);
	void mergeAnyCombination(const ChildSolutionMap& all_solutions, const SolutionBase& current,
	                         const moveit::core::RobotState& state, const Spawner& spawner);
	void merge(const ChildSolutionList& sub_solutions, const moveit::core::RobotState& state, const Spawner& spawner);

	void sendForward(SubTrajectory&& t, const InterfaceState* from);
	void sendBackward(SubTrajectory&& t, const InterfaceState* to);
	void spawn(SubTrajectory&& t);
};
PIMPL_FUNCTIONS(Merger)

} }
