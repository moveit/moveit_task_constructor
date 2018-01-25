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
#include "utils.h"
#include "stage_p.h"

#include <map>
#include <climits>

namespace moveit { namespace task_constructor {

/* A container needs to decouple its interfaces from those of its children.
 * Both, the container and the children have their own starts_ and ends_.
 * The container needs to forward states received in its interfaces to
 * the interfaces of the children.
 * Solutions found by the children, then need to be connected to the
 * container's interface states. To this end, we remember the mapping
 * from internal to external states.
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
	 * Negative index counts from end().
	 * Contrary to std::advance(), iterator limits are considered. */
	const_iterator position(int index) const;

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

protected:
	ContainerBasePrivate(ContainerBase *me, const std::string &name)
	   : StagePrivate(me, name)
	{}

	/// copy external_state to a child's interface and remember the link in internal_to map
	void copyState(InterfaceState &external_state, Stage &child, bool to_start);

protected:
	container_type children_;

	// map first child's start states to the corresponding states in this' starts_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_starts_;
	// map last child's end states to the corresponding states in this' ends_
	std::map<const InterfaceState*, InterfaceState*> internal_to_my_ends_;
};
PIMPL_FUNCTIONS(ContainerBase)

/** Representation of a single, full solution path of a SerialContainer.
 *
 * A serial solution describes a full solution path through all children
 * of a SerialContainer. This is a vector (of children().size()) of pointers
 * to all solutions of the children. Hence, we don't need to copy those solutions. */
class SerialSolution : public SolutionBase {
public:
	explicit SerialSolution(StagePrivate* creator, SerialContainer::solution_container&& subsolutions, double cost)
	   : SolutionBase(creator, cost), subsolutions_(subsolutions)
	{}
	/// append all subsolutions to solution
	void fillMessage(moveit_task_constructor_msgs::Solution &msg, Introspection *introspection) const override;

private:
	/// series of sub solutions
	SerialContainer::solution_container subsolutions_;
};


/* A solution of a SerialContainer needs to connect start to end via a full path.
 * The solution of a single child stage is usually disconnected to the container's start or end.
 * Only if all the children in the chain have found a coherent solution from start to end,
 * this solution can be announced as a solution of the SerialContainer.
 *
 * Particularly, the first/last stage's sendBackward()/sendForward() call
 * cannot directly propagate their associated state to the previous/next stage of this container,
 * because we cannot provide a full solution yet. Hence, the first/last stage
 * propagate to the pending_backward_/pending_forward_ interface first.
 * If eventually a full solution is found, it is propagated to prevEnds()/nextStarts() -
 * together with the solution. */
class SerialContainerPrivate : public ContainerBasePrivate {
	friend class SerialContainer;

public:
	SerialContainerPrivate(SerialContainer* me, const std::string &name);

	void storeNewSolution(SerialContainer::solution_container &&s, double cost);
	const ordered<SerialSolution>& solutions() const { return solutions_; }

private:
	void connect(StagePrivate *prev, StagePrivate *next);

	// interface to buffer first child's sendBackward() states
	InterfacePtr pending_backward_;
	// interface to buffer last child's sendForward() states
	InterfacePtr pending_forward_;

	// set of all solutions
	ordered<SerialSolution> solutions_;
};
PIMPL_FUNCTIONS(SerialContainer)


class ParallelContainerBasePrivate : public ContainerBasePrivate {
	friend class ParallelContainerBase;

public:
	ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string &name);
};
PIMPL_FUNCTIONS(ParallelContainerBase)


class WrapperBasePrivate : public ContainerBasePrivate {
	friend class WrapperBase;

public:
	WrapperBasePrivate(WrapperBase* me, const std::string& name);

private:
	InterfacePtr dummy_starts_;
	InterfacePtr dummy_ends_;
};
PIMPL_FUNCTIONS(WrapperBase)


class WrapperPrivate : public WrapperBasePrivate {
	friend class Wrapper;

public:
	WrapperPrivate(Wrapper* me, const std::string& name);

private:
	ordered<std::unique_ptr<SolutionBase>, pointerLessThan<std::unique_ptr<SolutionBase>>> solutions_;
	std::list<std::unique_ptr<SolutionBase>> failures_;
	std::list<InterfaceState> failure_states_;
};
PIMPL_FUNCTIONS(Wrapper)

} }
