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

/* Authors: Robert Haschke */

#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/task_constructor/merge.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ros/console.h>

#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/format.hpp>
#include <functional>

using namespace std::placeholders;

namespace moveit {
namespace task_constructor {

ContainerBasePrivate::ContainerBasePrivate(ContainerBase* me, const std::string& name) : StagePrivate(me, name) {
	pending_backward_.reset(new Interface);
	pending_forward_.reset(new Interface);
}

ContainerBasePrivate::const_iterator ContainerBasePrivate::childByIndex(int index, bool for_insert) const {
	if (!for_insert && index < 0)
		--index;
	const_iterator position = children_.begin();
	if (index > 0) {
		for (auto end = children_.end(); index > 0 && position != end; --index)
			++position;
	} else if (++index <= 0) {
		container_type::const_reverse_iterator from_end = children_.rbegin();
		for (auto end = children_.rend(); index < 0 && from_end != end; ++index)
			++from_end;
		position = index < 0 ? children_.end() : from_end.base();
	}
	return position;
}

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback& processor, unsigned int cur_depth,
                                          unsigned int max_depth) const {
	if (cur_depth >= max_depth)
		return true;

	for (auto& stage : children_) {
		if (!processor(*stage, cur_depth))
			continue;
		const ContainerBasePrivate* container = dynamic_cast<const ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, cur_depth + 1, max_depth);
	}
	return true;
}

void ContainerBasePrivate::validateConnectivity() const {
	InitStageException errors;
	// recursively validate all children and accumulate errors
	for (const auto& child : children()) {
		try {
			child->pimpl()->validateConnectivity();
		} catch (InitStageException& e) {
			errors.append(e);
		}
	}
	if (errors)
		throw errors;
}

void ContainerBasePrivate::mismatchingInterface(InitStageException& errors, const StagePrivate& child,
                                                const InterfaceFlags mask) const {
	boost::format desc("%1% interface of '%2%' (%3%) doesn't match mine (%4%)");
	errors.push_back(*me(), (desc % (mask == START_IF_MASK ? "start" : "end") % child.name() %
	                         flowSymbol(child.interfaceFlags() & mask) % flowSymbol(interfaceFlags() & mask))
	                            .str());
}

bool ContainerBasePrivate::canCompute() const {
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->canCompute();
}

void ContainerBasePrivate::compute() {
	// call the method of the public interface
	static_cast<ContainerBase*>(me_)->compute();
}

void ContainerBasePrivate::copyState(Interface::iterator external, const InterfacePtr& target, bool updated) {
	// TODO: update internal's prio from external's new priority
	if (updated)
		return;

	// create a clone of external state within target interface (child's starts() or ends())
	auto internal = states_.insert(states_.end(), InterfaceState(*external));
	target->add(*internal);
	// and remember the mapping between them
	internal_to_external_.insert(std::make_pair(&*internal, &*external));
}

void ContainerBasePrivate::liftSolution(SolutionBasePtr solution, const InterfaceState* internal_from,
                                        const InterfaceState* internal_to) {
	if (!storeSolution(solution))
		return;

	auto findOrCreateExternal = [this](const InterfaceState* internal, bool& created) -> InterfaceState* {
		auto it = internal_to_external_.find(internal);
		if (it != internal_to_external_.end())
			return it->second;

		InterfaceState* external = &*states_.insert(states_.end(), InterfaceState(*internal));
		internal_to_external_.insert(std::make_pair(internal, external));
		created = true;
		return external;
	};
	bool created_from = false;
	bool created_to = false;
	InterfaceState* external_from = findOrCreateExternal(internal_from, created_from);
	InterfaceState* external_to = findOrCreateExternal(internal_to, created_to);

	// connect solution to start/end state
	solution->setStartState(*external_from);
	solution->setEndState(*external_to);

	// spawn created states in external interfaces
	if (created_from)
		prevEnds()->add(*external_from);
	if (created_to)
		nextStarts()->add(*external_to);

	newSolution(solution);
}

ContainerBase::ContainerBase(ContainerBasePrivate* impl) : Stage(impl) {}

size_t ContainerBase::numChildren() const {
	return pimpl()->children().size();
}

Stage* ContainerBase::findChild(const std::string& name) const {
	auto pos = name.find('/');
	const std::string first = name.substr(0, pos);
	for (const Stage::pointer& child : pimpl()->children())
		if (child->name() == first) {
			if (pos == std::string::npos)
				return child.get();
			else if (auto* parent = dynamic_cast<const ContainerBase*>(child.get()))
				return parent->findChild(name.substr(pos + 1));
		}
	return nullptr;
}

bool ContainerBase::traverseChildren(const ContainerBase::StageCallback& processor) const {
	return pimpl()->traverseStages(processor, 0, 1);
}
bool ContainerBase::traverseRecursively(const ContainerBase::StageCallback& processor) const {
	if (!processor(*this, 0))
		return false;
	return pimpl()->traverseStages(processor, 1, UINT_MAX);
}

bool ContainerBase::insert(Stage::pointer&& stage, int before) {
	StagePrivate* impl = stage->pimpl();
	if (impl->parent() != nullptr || !stage->solutions().empty() || !stage->failures().empty()) {
		ROS_ERROR("cannot re-parent stage");
		return false;
	}

	ContainerBasePrivate::const_iterator where = pimpl()->childByIndex(before, true);
	ContainerBasePrivate::iterator it = pimpl()->children_.insert(where, std::move(stage));
	impl->setHierarchy(this, it);
	return true;
}

bool ContainerBasePrivate::remove(ContainerBasePrivate::const_iterator pos) {
	if (pos == children_.end())
		return false;

	(*pos)->pimpl()->setHierarchy(nullptr, ContainerBasePrivate::iterator());
	children_.erase(pos);
	return true;
}

bool ContainerBase::remove(int pos) {
	return pimpl()->remove(pimpl()->childByIndex(pos, false));
}

bool ContainerBase::remove(Stage* child) {
	auto it = pimpl()->children_.begin(), end = pimpl()->children_.end();
	for (; it != end && it->get() != child; ++it)
		;
	return pimpl()->remove(it);
}

void ContainerBase::clear() {
	pimpl()->children_.clear();
}

void ContainerBase::reset() {
	auto impl = pimpl();

	// recursively reset children
	for (auto& child : impl->children())
		child->reset();

	// clear buffer interfaces
	impl->pending_backward_->clear();
	impl->pending_forward_->clear();
	// ... and state mapping
	impl->internal_to_external_.clear();

	Stage::reset();
}

void ContainerBase::init(const moveit::core::RobotModelConstPtr& robot_model) {
	auto impl = pimpl();
	auto& children = impl->children();

	Stage::init(robot_model);

	// we need to have some children to do the actual work
	if (children.empty())
		throw InitStageException(*this, "no children");

	// recursively init all children and accumulate errors
	InitStageException errors;
	for (auto& child : children) {
		try {
			child->init(robot_model);
		} catch (const Property::error& e) {
			std::ostringstream oss;
			oss << e.what();
			pimpl()->composePropertyErrorMsg(e.name(), oss);
			errors.push_back(*child, oss.str());
		} catch (InitStageException& e) {
			errors.append(e);
		}
	}

	if (errors)
		throw errors;
}

std::ostream& operator<<(std::ostream& os, const ContainerBase& container) {
	ContainerBase::StageCallback processor = [&os](const Stage& stage, int depth) -> bool {
		os << std::string(2 * depth, ' ') << *stage.pimpl() << std::endl;
		return true;
	};
	container.traverseRecursively(processor);
	return os;
}

struct SolutionCollector
{
	SolutionCollector(size_t max_depth) : max_depth(max_depth) {}

	void operator()(const SolutionSequence::container_type& trace, double cost) {
#ifndef NDEBUG
		// Traced path should not extend past container boundaries, i.e. trace.size() <= max_depth
		// However, as the Merging-Connect's solution may be composed of several subsolutions, we need to disregard those
		size_t len = trace.size();
		const StagePrivate* prev_creator = nullptr;
		for (const auto& s : trace) {
			if (s->creator() == prev_creator)
				--len;
			else
				prev_creator = s->creator();
		}
		assert(len <= max_depth);
#endif
		solutions.emplace_back(std::make_pair(trace, cost));
	}

	typedef std::list<std::pair<SolutionSequence::container_type, double>> SolutionCostPairs;
	SolutionCostPairs solutions;
	const size_t max_depth;
};

void updateStateCosts(const SolutionSequence::container_type& partial_solution_path,
                      const InterfaceState::Priority& prio) {
	for (const SolutionBase* solution : partial_solution_path) {
		// here it suffices to update the start state, because the end state is the start state
		// of the next solution (they are all connected)
		InterfaceState* state = const_cast<InterfaceState*>(solution->start());
		if (state->owner())
			state->owner()->updatePriority(state, prio);
	}
	// finally update the end state of the last solution
	if (partial_solution_path.empty())
		return;
	InterfaceState* state = const_cast<InterfaceState*>(partial_solution_path.back()->end());
	if (state->owner())
		state->owner()->updatePriority(state, prio);
}

void SerialContainer::onNewSolution(const SolutionBase& current) {
	auto impl = pimpl();
	const StagePrivate* creator = current.creator();
	auto& children = impl->children();

	// find number of stages before and after creator stage
	size_t num_before = 0, num_after = 0;
	for (auto it = children.begin(), end = children.end(); it != end; ++it, ++num_before)
		if ((*it)->pimpl() == creator)
			break;
	assert(num_before < children.size());  // creator should be one of our children
	num_after = children.size() - 1 - num_before;

	SolutionSequence::container_type trace;
	trace.reserve(children.size());

	// find all incoming solution paths ending at current solution
	SolutionCollector incoming(num_before);
	traverse<Interface::BACKWARD>(current, std::ref(incoming), trace);

	// find all outgoing solution paths starting at current solution
	SolutionCollector outgoing(num_after);
	traverse<Interface::FORWARD>(current, std::ref(outgoing), trace);

	// collect (and sort) all solutions spanning from start to end of this container
	ordered<SolutionSequencePtr> sorted;
	SolutionSequence::container_type solution;
	solution.reserve(children.size());
	for (auto& in : incoming.solutions) {
		for (auto& out : outgoing.solutions) {
			InterfaceState::Priority prio(in.first.size() + 1 + out.first.size(), in.second + current.cost() + out.second);
			// found a complete solution path connecting start to end?
			if (prio.depth() == children.size()) {
				if (std::isinf(prio.cost()))
					continue;  // don't propagate failures
				assert(solution.empty());
				// insert incoming solutions in reverse order
				solution.insert(solution.end(), in.first.rbegin(), in.first.rend());
				// insert current solution
				solution.push_back(&current);
				// insert outgoing solutions in normal order
				solution.insert(solution.end(), out.first.begin(), out.first.end());
				// store solution in sorted list
				sorted.insert(std::make_shared<SolutionSequence>(std::move(solution), prio.cost(), impl));
			} else if (prio.depth() > 1) {
				// update state priorities along the whole partial solution path
				updateStateCosts(in.first, prio);
				updateStateCosts({ &current }, prio);
				updateStateCosts(out.first, prio);
			}
		}
	}

	// finally store + announce new solutions to external interface
	for (const auto& solution : sorted)
		impl->liftSolution(solution, solution->internalStart(), solution->internalEnd());
}

SerialContainer::SerialContainer(SerialContainerPrivate* impl) : ContainerBase(impl) {}
SerialContainer::SerialContainer(const std::string& name) : SerialContainer(new SerialContainerPrivate(this, name)) {}

SerialContainerPrivate::SerialContainerPrivate(SerialContainer* me, const std::string& name)
  : ContainerBasePrivate(me, name) {}

// a serial container's required interface is derived from the required input interfaces
// of the first and last children. After resolving, it is remembered in required_interface_.
InterfaceFlags SerialContainerPrivate::requiredInterface() const {
	if ((required_interface_ & START_IF_MASK) && (required_interface_ & END_IF_MASK))
		return required_interface_;

	if (children().empty())
		return UNKNOWN;
	return (children().front()->pimpl()->requiredInterface() & START_IF_MASK) |
	       (children().back()->pimpl()->requiredInterface() & END_IF_MASK);
}

// connect cur stage to its predecessor and successor by setting the push interface pointers
// return true if cur stage should be scheduled for a second sweep
bool SerialContainerPrivate::connect(container_type::const_iterator cur) {
	StagePrivate* const cur_impl = **cur;
	InterfaceFlags required = cur_impl->requiredInterface();

	// get iterators to prev / next stage in sequence
	auto prev = cur;
	--prev;
	auto next = cur;
	++next;

	// set push forward connection using next's starts
	if ((required == UNKNOWN || required & WRITES_NEXT_START) &&
	    next != children().end())  // last child has not a next one
		cur_impl->setNextStarts((*next)->pimpl()->starts());

	// set push backward connection using prev's ends
	if ((required == UNKNOWN || required & WRITES_PREV_END) &&
	    cur != children().begin())  // first child has not a previous one
		cur_impl->setPrevEnds((*prev)->pimpl()->ends());

	// schedule stage with unknown interface for 2nd sweep
	return required == UNKNOWN || required == PROPAGATE_BOTHWAYS;
}

/* Establishing the interface connections, we face a chicken-egg-problem:
 * To establish a connection, a predecessors/successors pull interface is
 * assigned to the current's stage push interface.
 * However, propagating stages (in auto-detection mode) can only create
 * their pull interfaces if the corresponding, opposite-side push interface
 * is present already (because that's the mechanism to determine the supported
 * propagation directions).
 *
 * Hence, we need to resolve this by performing two sweeps:
 * - initialization, assuming both propagation directions should be supported,
 *   thus generating both pull interfaces, i.e. providing the egg
 * - stripping down the interfaces to the actual context
 *   This context is provided by two stages pushing from both ends
 *   into a (potentially long) sequence of propagating stages (tbd).
 */
void SerialContainer::init(const moveit::core::RobotModelConstPtr& robot_model) {
	// reset pull interfaces
	auto impl = pimpl();
	impl->starts_.reset();
	impl->ends_.reset();
	impl->required_interface_ = UNKNOWN;

	// recursively init all children, throws if there are no children
	ContainerBase::init(robot_model);

	auto start = impl->children().begin();
	auto last = --impl->children().end();

	// connect first / last child's push interfaces to our pending_* buffers
	// if they require pushing
	impl->setChildsPushBackwardInterface(**start);
	impl->setChildsPushForwardInterface(**last);

	// initialize and connect remaining children in two sweeps
	// to allow interface auto-detection for propagating stages
	auto first_unknown = start;  // pointer to first stage with unknown interface
	for (auto cur = start, end = impl->children().end(); cur != end; ++cur) {
		// 1st sweep: connect everything potentially possible,
		// remembering start of unknown sub sequence
		if (impl->connect(cur))
			;
		else {  // reached a stage with known interface
			// 2nd sweep: prune interfaces from [first_unknown, cur)
			impl->pruneInterfaces(first_unknown, cur);
			// restart with first_unknown = ++cur
			first_unknown = cur;
			++first_unknown;
		}
	}
	// prune stages [first_unknown, end())
	impl->pruneInterfaces(first_unknown, impl->children().end());

	// initialize this' pull interfaces if first/last child pulls
	if (const InterfacePtr& target = (*start)->pimpl()->starts())
		impl->starts_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, std::placeholders::_1,
		                                            std::cref(target), std::placeholders::_2)));
	if (const InterfacePtr& target = (*last)->pimpl()->ends())
		impl->ends_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, std::placeholders::_1,
		                                          std::cref(target), std::placeholders::_2)));
}

// prune interface for children in range [first, last) to given direction
void SerialContainerPrivate::storeRequiredInterface(container_type::const_iterator first,
                                                    container_type::const_iterator end) {
	if (first == children().begin())
		required_interface_ |= children().front()->pimpl()->interfaceFlags() & START_IF_MASK;
	if (end == children().end() && !children().empty())
		required_interface_ |= children().back()->pimpl()->interfaceFlags() & END_IF_MASK;
}

// called by parent asking for pruning of this' interface
void SerialContainerPrivate::pruneInterface(InterfaceFlags accepted) {
	if (children().empty())
		return;

	// reading is always allowed if current interface flags do so
	accepted |= (interfaceFlags() & InterfaceFlags({ READS_START, READS_END }));

	if (accepted == PROPAGATE_BOTHWAYS)
		return;  // There is nothing to prune

	// If whole chain is still undecided, prune all children
	if (children().front()->pimpl()->interfaceFlags() == PROPAGATE_BOTHWAYS &&
	    children().back()->pimpl()->interfaceFlags() == PROPAGATE_BOTHWAYS) {
		pruneInterfaces(children().begin(), children().end(), accepted);
	} else {  // otherwise only prune the first / last child's input / output interface
		StagePrivate* child_impl;
		child_impl = children().front()->pimpl();
		child_impl->pruneInterface((accepted & START_IF_MASK) | (child_impl->interfaceFlags() & END_IF_MASK));
		child_impl = children().back()->pimpl();
		child_impl->pruneInterface((accepted & END_IF_MASK) | (child_impl->interfaceFlags() & START_IF_MASK));
	}

	// reset my pull interfaces, if first/last child don't pull anymore
	if (!children().front()->pimpl()->starts())
		starts_.reset();
	if (!children().back()->pimpl()->ends())
		ends_.reset();

	if (interfaceFlags() == UNKNOWN)
		throw InitStageException(*me(), "failed to derive propagation direction");
}

// called by init() to prune interfaces for children in range [first, last)
// this function determines the feasible propagation directions
void SerialContainerPrivate::pruneInterfaces(container_type::const_iterator first, container_type::const_iterator end) {
	if (first == end) {
		storeRequiredInterface(first, end);
		return;  // nothing to do in this case
	}

	// determine accepted interface from available push interfaces
	InterfaceFlags accepted;

	// if first stage ...
	if (first != children().begin()) {
		auto prev = first;
		--prev;  // pointer to previous stage
		// ... pushes forward, we accept forward propagation
		if ((*prev)->pimpl()->requiredInterface() & WRITES_NEXT_START)
			accepted |= PROPAGATE_FORWARDS;
		// ... pulls backward, we accept backward propagation
		if ((*prev)->pimpl()->requiredInterface() & READS_END)
			accepted |= PROPAGATE_BACKWARDS;
	}  // else: for first child we cannot determine the interface yet

	// if end stage ...
	if (end != children().end()) {
		// ... pushes backward, we accept backward propagation
		if ((*end)->pimpl()->requiredInterface() & WRITES_PREV_END)
			accepted |= PROPAGATE_BACKWARDS;
		// ... pulls forward, we accept forward propagation
		if ((*end)->pimpl()->requiredInterface() & READS_START)
			accepted |= PROPAGATE_FORWARDS;
	}  // else: for last child we cannot determine the interface yet

	// nothing to do if:
	// - accepted == UNKNOWN: interface still unknown
	// - accepted == PROPAGATE_BOTHWAYS: no change
	if (accepted != UNKNOWN && accepted != PROPAGATE_BOTHWAYS)
		pruneInterfaces(first, end, accepted);
}

// prune interface for children in range [first, last) to given direction
void SerialContainerPrivate::pruneInterfaces(container_type::const_iterator first, container_type::const_iterator end,
                                             InterfaceFlags accepted) {
	// 1st sweep: remove push interfaces
	for (auto it = first; it != end; ++it) {
		StagePrivate* impl = (*it)->pimpl();
		// the required interface should be a subset of the accepted one
		if ((impl->requiredInterface() & accepted) != impl->requiredInterface())
			throw InitStageException(*impl->me(), "Required interface not satisfied after pruning");

		// remove push interfaces if not accepted
		if (!(accepted & WRITES_PREV_END))
			impl->setPrevEnds(InterfacePtr());

		if (!(accepted & WRITES_NEXT_START))
			impl->setNextStarts(InterfacePtr());
	}
	// 2nd sweep: recursively prune children
	for (auto it = first; it != end; ++it) {
		StagePrivate* impl = (*it)->pimpl();
		impl->pruneInterface(accepted);
	}

	storeRequiredInterface(first, end);
}

void SerialContainerPrivate::validateConnectivity() const {
	InitStageException errors;

	// recursively validate children
	try {
		ContainerBasePrivate::validateConnectivity();
	} catch (InitStageException& e) {
		errors.append(e);
	}

	// check that input / output interface of first / last child matches this' resp. interface
	if (!children().empty()) {
		const StagePrivate* start = children().front()->pimpl();
		const auto my_flags = this->interfaceFlags();
		auto child_flags = start->interfaceFlags() & START_IF_MASK;
		if (child_flags != (my_flags & START_IF_MASK))
			mismatchingInterface(errors, *start, START_IF_MASK);

		const StagePrivate* last = children().back()->pimpl();
		child_flags = last->interfaceFlags() & END_IF_MASK;
		if (child_flags != (my_flags & END_IF_MASK))
			mismatchingInterface(errors, *last, END_IF_MASK);
	}

	// validate connectivity of children amongst each other
	// ContainerBasePrivate::validateConnectivity() ensures that required push interfaces are present,
	// that is, neighbouring stages have a corresponding pull interface.
	// Here, it remains to check that - if a child has a pull interface - it's indeed feeded.
	for (auto cur = children().begin(), end = children().end(); cur != end; ++cur) {
		const StagePrivate* const cur_impl = **cur;
		InterfaceFlags required = cur_impl->interfaceFlags();

		// get iterators to prev / next stage in sequence
		auto prev = cur;
		--prev;
		auto next = cur;
		++next;

		// start pull interface fed?
		if (cur != children().begin() &&  // first child has not a previous one
		    (required & READS_START) && !(*prev)->pimpl()->nextStarts())
			errors.push_back(**cur, "start interface is not fed");

		// end pull interface fed?
		if (next != end &&  // last child has not a next one
		    (required & READS_END) && !(*next)->pimpl()->prevEnds())
			errors.push_back(**cur, "end interface is not fed");
	}

	if (errors)
		throw errors;
}

bool SerialContainer::canCompute() const {
	for (const auto& stage : pimpl()->children()) {
		if (stage->pimpl()->canCompute())
			return true;
	}
	return false;
}

void SerialContainer::compute() {
	for (const auto& stage : pimpl()->children()) {
		try {
			if (!stage->pimpl()->canCompute())
				continue;

			ROS_DEBUG("Computing stage '%s'", stage->name().c_str());
			stage->pimpl()->runCompute();
		} catch (const Property::error& e) {
			stage->reportPropertyError(e);
		}
	}
}

template <Interface::Direction dir>
void SerialContainer::traverse(const SolutionBase& start, const SolutionProcessor& cb,
                               SolutionSequence::container_type& trace, double trace_cost) {
	const InterfaceState::Solutions& solutions = start.trajectories<dir>();
	if (solutions.empty())  // if we reached the end, call the callback
		cb(trace, trace_cost);
	else
		for (SolutionBase* successor : solutions) {
			trace.push_back(successor);
			trace_cost += successor->cost();

			traverse<dir>(*successor, cb, trace, trace_cost);

			trace_cost -= successor->cost();
			trace.pop_back();
		}
}

void WrappedSolution::fillMessage(moveit_task_constructor_msgs::Solution& solution,
                                  Introspection* introspection) const {
	wrapped_->fillMessage(solution, introspection);

	// prepend this solutions info as a SubSolution msg
	moveit_task_constructor_msgs::SubSolution sub_msg;
	SolutionBase::fillInfo(sub_msg.info, introspection);
	sub_msg.sub_solution_id.push_back(introspection ? introspection->solutionId(*wrapped_) : 0);
	solution.sub_solution.insert(solution.sub_solution.begin(), std::move(sub_msg));
}

ParallelContainerBasePrivate::ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string& name)
  : ContainerBasePrivate(me, name) {}

// A parallel container's required interface is derived from the required interfaces of all of its children.
// They must not conflict to each other. Otherwise an InitStageException is thrown.
InterfaceFlags ParallelContainerBasePrivate::requiredInterface() const {
	if (children().empty())
		return UNKNOWN;
	/* The interfaces of all children need to be consistent with each other. Allowed combinations are:
	 * ❘ ❘ = ❘  (connecting stages)
	 * ↑ ↑ = ↑  (backward propagating)
	 * ↓ ↓ = ↓  (forward propagating)
	 * ↑ ↓ = ⇅ = ⇅ ↑ = ⇅ ↓  (propagating in both directions)
	 * ↕ ↕ = ↕  (generating)
	 */

	InterfaceFlags accumulated = children().front()->pimpl()->requiredInterface();
	for (const Stage::pointer& stage : children()) {
		InterfaceFlags current = stage->pimpl()->requiredInterface();
		if (accumulated != PROPAGATE_BOTHWAYS &&
		    (accumulated & current) == current)  // all flags of current are already available in accumulated
			continue;

		bool current_is_propagating =
		    (current == PROPAGATE_BOTHWAYS || current == PROPAGATE_FORWARDS || current == PROPAGATE_BACKWARDS);

		if (current_is_propagating && accumulated != CONNECT && accumulated != GENERATE)
			accumulated |= current;  // propagating is compatible to all except CONNECT and GENERATE
		else
			throw InitStageException(*me(),
			                         "child '" + stage->name() + "' has conflicting interface to previous children");
	}
	return accumulated;
}

void ParallelContainerBasePrivate::pruneInterface(InterfaceFlags accepted) {
	// forward pruning to all children with UNKNOWN required interface
	for (const Stage::pointer& stage : children()) {
		if (stage->pimpl()->requiredInterface() == UNKNOWN)
			stage->pimpl()->pruneInterface(accepted);
	}
}

void ParallelContainerBasePrivate::validateConnectivity() const {
	InitStageException errors;
	InterfaceFlags my_interface = interfaceFlags();
	InterfaceFlags children_interfaces;

	// check that input / output interfaces of all children are handled by my interface
	for (const auto& child : children()) {
		InterfaceFlags current = child->pimpl()->interfaceFlags();
		children_interfaces |= current;  // compute union of all children interfaces

		if ((current & my_interface & START_IF_MASK) != (current & START_IF_MASK))
			mismatchingInterface(errors, *child->pimpl(), START_IF_MASK);
		if ((current & my_interface & END_IF_MASK) != (current & END_IF_MASK))
			mismatchingInterface(errors, *child->pimpl(), END_IF_MASK);
	}
	// check that there is a child matching the expected push interfaces
	if ((my_interface & GENERATE) != (children_interfaces & GENERATE))
		errors.push_back(*me(), "no child provides expected push interface");

	// recursively validate children
	try {
		ContainerBasePrivate::validateConnectivity();
	} catch (InitStageException& e) {
		errors.append(e);
	}

	if (errors)
		throw errors;
}

void ParallelContainerBasePrivate::onNewExternalState(Interface::Direction dir, Interface::iterator external,
                                                      bool updated) {
	for (const Stage::pointer& stage : children())
		copyState(external, stage->pimpl()->pullInterface(dir), updated);
}

ParallelContainerBase::ParallelContainerBase(ParallelContainerBasePrivate* impl) : ContainerBase(impl) {}
ParallelContainerBase::ParallelContainerBase(const std::string& name)
  : ParallelContainerBase(new ParallelContainerBasePrivate(this, name)) {}

/* States received by the container need to be copied to all children's pull interfaces.
 * States generated by children can be directly forwarded into the container's push interfaces.
 */
void ParallelContainerBase::init(const moveit::core::RobotModelConstPtr& robot_model) {
	// recursively init children
	ContainerBase::init(robot_model);
	auto impl = pimpl();

	// determine the union of interfaces required by children
	// TODO: should we better use the least common interface?
	InterfaceFlags required = impl->requiredInterface();

	// initialize this' pull connections
	impl->starts().reset(required & READS_START ?
	                         new Interface(std::bind(&ParallelContainerBasePrivate::onNewExternalState, impl,
	                                                 Interface::FORWARD, std::placeholders::_1, std::placeholders::_2)) :
	                         nullptr);
	impl->ends().reset(required & READS_END ?
	                       new Interface(std::bind(&ParallelContainerBasePrivate::onNewExternalState, impl,
	                                               Interface::BACKWARD, std::placeholders::_1, std::placeholders::_2)) :
	                       nullptr);

	// initialize push connections of children according to their demands
	for (const Stage::pointer& stage : impl->children()) {
		impl->setChildsPushForwardInterface(*stage);
		impl->setChildsPushBackwardInterface(*stage);
	}
}

void ParallelContainerBase::liftSolution(const SolutionBase& solution, double cost, std::string comment) {
	auto impl = pimpl();
	impl->liftSolution(std::make_shared<WrappedSolution>(impl, &solution, cost, std::move(comment)), solution.start(),
	                   solution.end());
}

void ParallelContainerBase::spawn(InterfaceState&& state, SubTrajectory&& t) {
	pimpl()->StagePrivate::spawn(std::move(state), std::make_shared<SubTrajectory>(std::move(t)));
}

void ParallelContainerBase::sendForward(const InterfaceState& from, InterfaceState&& to, SubTrajectory&& t) {
	pimpl()->StagePrivate::sendForward(from, std::move(to), std::make_shared<SubTrajectory>(std::move(t)));
}

void ParallelContainerBase::sendBackward(InterfaceState&& from, const InterfaceState& to, SubTrajectory&& t) {
	pimpl()->StagePrivate::sendBackward(std::move(from), to, std::make_shared<SubTrajectory>(std::move(t)));
}

WrapperBasePrivate::WrapperBasePrivate(WrapperBase* me, const std::string& name)
  : ParallelContainerBasePrivate(me, name) {}

WrapperBase::WrapperBase(const std::string& name, Stage::pointer&& child)
  : WrapperBase(new WrapperBasePrivate(this, name), std::move(child)) {}

WrapperBase::WrapperBase(WrapperBasePrivate* impl, Stage::pointer&& child) : ParallelContainerBase(impl) {
	if (child)
		WrapperBase::insert(std::move(child));
}

bool WrapperBase::insert(Stage::pointer&& stage, int before) {
	// restrict num of children to one
	if (numChildren() > 0)
		return false;
	return ParallelContainerBase::insert(std::move(stage), before);
}

Stage* WrapperBase::wrapped() {
	return pimpl()->children().empty() ? nullptr : pimpl()->children().front().get();
}

bool WrapperBase::canCompute() const {
	return wrapped()->pimpl()->canCompute();
}

void WrapperBase::compute() {
	try {
		wrapped()->pimpl()->runCompute();
	} catch (const Property::error& e) {
		wrapped()->reportPropertyError(e);
	}
}

bool Alternatives::canCompute() const {
	for (const auto& stage : pimpl()->children())
		if (stage->pimpl()->canCompute())
			return true;
	return false;
}

void Alternatives::compute() {
	for (const auto& stage : pimpl()->children()) {
		try {
			stage->pimpl()->runCompute();
		} catch (const Property::error& e) {
			stage->reportPropertyError(e);
		}
	}
}

void Alternatives::onNewSolution(const SolutionBase& s) {
	liftSolution(s);
}

void Fallbacks::reset() {
	active_child_ = nullptr;
	ParallelContainerBase::reset();
}

void Fallbacks::init(const moveit::core::RobotModelConstPtr& robot_model) {
	ParallelContainerBase::init(robot_model);
	active_child_ = pimpl()->children().front().get();
}

bool Fallbacks::canCompute() const {
	while (active_child_) {
		StagePrivate* child = active_child_->pimpl();
		if (child->canCompute())
			return true;

		// active child failed, continue with next
		auto next = child->it();
		++next;
		active_child_ = next->get();
	}
	return false;
}

void Fallbacks::compute() {
	if (!active_child_)
		return;

	try {
		active_child_->pimpl()->runCompute();
	} catch (const Property::error& e) {
		active_child_->reportPropertyError(e);
	}
}

void Fallbacks::onNewSolution(const SolutionBase& s) {
	liftSolution(s);
}

MergerPrivate::MergerPrivate(Merger* me, const std::string& name) : ParallelContainerBasePrivate(me, name) {}

InterfaceFlags MergerPrivate::requiredInterface() const {
	if (children().size() < 2)
		throw InitStageException(*me_, "Need 2 children at least.");

	InterfaceFlags required = ParallelContainerBasePrivate::requiredInterface();

	// all children need to share a common interface
	for (const Stage::pointer& stage : children()) {
		InterfaceFlags current = stage->pimpl()->requiredInterface();
		if (current != required)
			throw InitStageException(*stage, "Interface doesn't match the common one.");
	}

	switch (required) {
		case PROPAGATE_FORWARDS:
		case PROPAGATE_BACKWARDS:
		case UNKNOWN:
			break;  // these are supported
		case GENERATE:
			throw InitStageException(*me_, "Generator stages not yet supported.");
		case CONNECT:
			throw InitStageException(*me_, "Cannot merge connecting stages. Use Connect.");
		default:
			throw InitStageException(*me_, "Children's interface not supported.");
	}
	return required;
}

Merger::Merger(const std::string& name) : Merger(new MergerPrivate(this, name)) {}

void Merger::reset() {
	ParallelContainerBase::reset();
	auto impl = pimpl();
	impl->jmg_merged_.reset();
	impl->source_state_to_solutions_.clear();
}

void Merger::init(const core::RobotModelConstPtr& robot_model) {
	ParallelContainerBase::init(robot_model);
}

Merger::Merger(MergerPrivate* impl) : ParallelContainerBase(impl) {}

bool Merger::canCompute() const {
	for (const auto& stage : pimpl()->children())
		if (stage->pimpl()->canCompute())
			return true;
	return false;
}

void Merger::compute() {
	for (const auto& stage : pimpl()->children()) {
		try {
			stage->pimpl()->runCompute();
		} catch (const Property::error& e) {
			stage->reportPropertyError(e);
		}
	}
}

void Merger::onNewSolution(const SolutionBase& s) {
	auto impl = pimpl();
	switch (impl->interfaceFlags()) {
		case PROPAGATE_FORWARDS:
		case PROPAGATE_BACKWARDS:
			impl->onNewPropagateSolution(s);
			break;
		case GENERATE:
			impl->onNewGeneratorSolution(s);
			break;
		default:
			assert(false);
	}
}

void MergerPrivate::onNewPropagateSolution(const SolutionBase& s) {
	const SubTrajectory* trajectory = dynamic_cast<const SubTrajectory*>(&s);
	if (!trajectory) {
		ROS_ERROR_NAMED("Merger", "Only simple trajectories are supported");
		return;
	}

	InterfaceFlags dir = interfaceFlags();
	assert(dir == PROPAGATE_FORWARDS || dir == PROPAGATE_BACKWARDS);
	// internal source state
	const InterfaceState* source_state = (dir == PROPAGATE_FORWARDS) ? s.start() : s.end();

	// map to external source state that is shared by all children
	auto source_it = internalToExternalMap().find(source_state);
	// internal->external mapping for source state should have been created
	assert(source_it != internalToExternalMap().end());
	InterfaceState* external_source_state = &*source_it->second;

	// retrieve (or create if necessary) the ChildSolutionMap for the given external source state
	ChildSolutionMap& all_solutions =
	    source_state_to_solutions_.insert(std::make_pair(external_source_state, ChildSolutionMap())).first->second;

	// retrieve (or create if necessary) the ChildSolutionList corresponding to the child
	ChildSolutionList& child_solutions =
	    all_solutions.insert(std::make_pair(s.creator(), ChildSolutionList())).first->second;
	// insert the new child solution into the list
	child_solutions.push_back(trajectory);

	// do we have solutions for all children?
	if (all_solutions.size() < children().size())
		return;
	assert(all_solutions.size() == children().size());

	// combine the new solution with all solutions from other children
	auto spawner = dir == PROPAGATE_FORWARDS ? &MergerPrivate::sendForward : &MergerPrivate::sendBackward;
	mergeAnyCombination(all_solutions, s, external_source_state->scene(),
	                    std::bind(spawner, this, std::placeholders::_1, external_source_state));
}

void MergerPrivate::sendForward(SubTrajectory&& t, const InterfaceState* from) {
	// generate target state
	planning_scene::PlanningScenePtr to = from->scene()->diff();
	to->setCurrentState(t.trajectory()->getLastWayPoint());
	StagePrivate::sendForward(*from, InterfaceState(to), std::make_shared<SubTrajectory>(std::move(t)));
}

void MergerPrivate::sendBackward(SubTrajectory&& t, const InterfaceState* to) {
	// generate target state
	planning_scene::PlanningScenePtr from = to->scene()->diff();
	from->setCurrentState(t.trajectory()->getFirstWayPoint());
	StagePrivate::sendBackward(InterfaceState(from), *to, std::make_shared<SubTrajectory>(std::move(t)));
}

void MergerPrivate::onNewGeneratorSolution(const SolutionBase& s) {
	// TODO: implement in similar fashion as onNewPropagateSolution(), but also merge start/end states
}

void MergerPrivate::mergeAnyCombination(const ChildSolutionMap& all_solutions, const SolutionBase& current,
                                        const planning_scene::PlanningSceneConstPtr& start_scene,
                                        const Spawner& spawner) {
	std::vector<size_t> indeces;  // which solution index was considered last for i-th child?
	indeces.reserve(children().size());

	ChildSolutionList sub_solutions;
	sub_solutions.reserve(children().size());

	// initialize vector of sub solutions
	for (const auto& pair : all_solutions) {
		// all children, except current solution's creator, start with zero index
		indeces.push_back(pair.first != current.creator() ? 0 : pair.second.size() - 1);
		sub_solutions.push_back(pair.second[indeces.back()]);
	}
	while (true) {
		merge(sub_solutions, start_scene, spawner);

		// compose next combination
		size_t child = 0;
		for (auto it = all_solutions.cbegin(), end = all_solutions.cend(); it != end; ++it, ++child) {
			if (it->first == current.creator())
				continue;  // skip current solution's child
			if (++indeces[child] >= it->second.size()) {
				indeces[child] = 0;  // start over with zero
				sub_solutions[child] = it->second[indeces[child]];
				continue;  // and continue with next child
			}
			// otherwise, a new solution combination is available
			sub_solutions[child] = it->second[indeces[child]];
			break;
		}
		if (child == children().size())  // all combinations exhausted?
			break;
	}
}

void MergerPrivate::merge(const ChildSolutionList& sub_solutions,
                          const planning_scene::PlanningSceneConstPtr& start_scene, const Spawner& spawner) {
	// transform vector of SubTrajectories into vector of RobotTrajectories
	std::vector<robot_trajectory::RobotTrajectoryConstPtr> sub_trajectories;
	sub_trajectories.reserve(sub_solutions.size());
	for (const auto& sub : sub_solutions) {
		// TODO: directly skip failures in mergeAnyCombination() or even earlier
		if (sub->isFailure())
			return;
		if (sub->trajectory())
			sub_trajectories.push_back(sub->trajectory());
	}

	moveit::core::JointModelGroup* jmg = jmg_merged_.get();
	robot_trajectory::RobotTrajectoryPtr merged =
	    task_constructor::merge(sub_trajectories, start_scene->getCurrentState(), jmg);
	if (jmg_merged_.get() != jmg)
		jmg_merged_.reset(jmg);
	if (!merged)
		return;

	// check merged trajectory for collisions
	if (!start_scene->isPathValid(*merged))
		return;

	SubTrajectory t(merged);
	// accumulate costs and markers
	double costs = 0.0;
	for (const auto& sub : sub_solutions) {
		costs += sub->cost();
		t.markers().insert(t.markers().end(), sub->markers().begin(), sub->markers().end());
	}
	t.setCost(costs);
	spawner(std::move(t));
}
}
}
