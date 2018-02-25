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
#include <ros/console.h>

#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/range/adaptor/reversed.hpp>
#include <functional>

using namespace std::placeholders;

namespace moveit { namespace task_constructor {

ContainerBasePrivate::ContainerBasePrivate(ContainerBase *me, const std::string &name)
   : StagePrivate(me, name)
{
	pending_backward_.reset(new Interface);
	pending_forward_.reset(new Interface);
}

ContainerBasePrivate::const_iterator ContainerBasePrivate::position(int index) const {
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

bool ContainerBasePrivate::traverseStages(const ContainerBase::StageCallback &processor,
                                          unsigned int cur_depth, unsigned int max_depth) const {
	if (cur_depth >= max_depth)
		return true;

	for (auto &stage : children_) {
		if (!processor(*stage, cur_depth))
			continue;
		const ContainerBasePrivate *container = dynamic_cast<const ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, cur_depth+1, max_depth);
	}
	return true;
}

bool ContainerBasePrivate::canCompute() const
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->canCompute();
}

bool ContainerBasePrivate::compute()
{
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->compute();
}

void ContainerBasePrivate::copyState(Interface::iterator external, const InterfacePtr& target, bool updated) {
	// TODO need to update existing mapping?

	// create a clone of external state within target interface (child's starts() or ends())
	InterfaceState& internal = *target->clone(*external);
	// and remember the mapping between them
	internal_to_external_.insert(std::make_pair(&internal, external));
}

void ContainerBasePrivate::liftSolution(SolutionBase& solution,
                                        const InterfaceState *internal_from, const InterfaceState *internal_to)
{
	// add solution to existing or new start state
	auto it = internal_to_external_.find(internal_from);
	if (it != internal_to_external_.end()) {
		// connect solution to existing start state
		solution.setStartState(*it->second);
	} else {
		// spawn a new state in previous stage
		Interface::iterator external = prevEnds()->add(InterfaceState(*internal_from), NULL, &solution);
		internal_to_external_.insert(std::make_pair(internal_from, external));
	}

	// add solution to existing or new end state
	it = internal_to_external_.find(internal_to);
	if (it != internal_to_external_.end()) {
		// connect solution to existing start state
		solution.setEndState(*it->second);
	} else {
		// spawn a new state in next stage
		Interface::iterator external = nextStarts()->add(InterfaceState(*internal_to), &solution, NULL);
		internal_to_external_.insert(std::make_pair(internal_to, external));
	}
}


ContainerBase::ContainerBase(ContainerBasePrivate *impl)
   : Stage(impl)
{
}

size_t ContainerBase::numChildren() const
{
	return pimpl()->children().size();
}

bool ContainerBase::traverseChildren(const ContainerBase::StageCallback &processor) const
{
	return pimpl()->traverseStages(processor, 0, 1);
}
bool ContainerBase::traverseRecursively(const ContainerBase::StageCallback &processor) const
{
	if (!processor(*this, 0))
		return false;
	return pimpl()->traverseStages(processor, 1, UINT_MAX);
}

bool ContainerBase::insert(Stage::pointer &&stage, int before)
{
	StagePrivate *impl = stage->pimpl();
	if (impl->parent() != nullptr || numSolutions() != 0) {
		ROS_ERROR("cannot re-parent stage");
		return false;
	}

	ContainerBasePrivate::const_iterator where = pimpl()->position(before);
	ContainerBasePrivate::iterator it = pimpl()->children_.insert(where, std::move(stage));
	impl->setHierarchy(this, it);
	return true;
}

bool ContainerBase::remove(int pos)
{
	ContainerBasePrivate::const_iterator it = pimpl()->position(pos);
	(*it)->pimpl()->setHierarchy(nullptr, ContainerBasePrivate::iterator());
	pimpl()->children_.erase(it);
	return true;
}

void ContainerBase::clear()
{
	pimpl()->children_.clear();
}

void ContainerBase::exposePropertiesOfChild(int child, const std::initializer_list<std::string>& names)
{
	auto impl = pimpl();
	// for negative child index, return last child for -1, next to last for -2, etc
	ContainerBasePrivate::const_iterator child_it = impl->position(child < 0 ? child-1 : child);
	if (child_it == impl->children().end())
		throw std::runtime_error("invalid child index");

	auto &child_props = (*child_it)->properties();
	// declare variables
	child_props.exposeTo(impl->properties_, names);
	// configure inheritance
	child_props.configureInitFrom(Stage::PARENT, names);
}

void ContainerBase::exposePropertyOfChildAs(int child, const std::string& child_property_name,
                                            const std::string& parent_property_name)
{
	auto impl = pimpl();
	// for negative child index, return last child for -1, next to last for -2, etc
	ContainerBasePrivate::const_iterator child_it = impl->position(child < 0 ? child-1 : child);
	if (child_it == impl->children().end())
		throw std::runtime_error("invalid child index");

	auto &child_props = (*child_it)->properties();
	// declare variables
	child_props.exposeTo(impl->properties_, child_property_name, parent_property_name);
	// configure inheritance
	child_props.property(child_property_name).configureInitFrom(Stage::PARENT, parent_property_name);
}

void ContainerBase::reset()
{
	auto impl = pimpl();

	// recursively reset children
	for (auto& child: impl->children())
		child->reset();

	// clear buffer interfaces
	impl->pending_backward_->clear();
	impl->pending_forward_->clear();
	// ... and state mapping
	impl->internal_to_external_.clear();

	Stage::reset();
}

void ContainerBase::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	auto impl = pimpl();
	auto& children = impl->children();

	Stage::init(robot_model);

	// we need to have some children to do the actual work
	if (children.empty())
		throw InitStageException(*this, "no children");

	// recursively init all children and accumulate errors
	InitStageException errors;
	for (auto& child : children) {
		try { child->init(robot_model); } catch (InitStageException &e) { errors.append(e); }
	}

	if (errors) throw errors;
}

void ContainerBase::validateConnectivity() const
{
	InitStageException errors;
	for (const auto& child : pimpl()->children()) {
		// check that child's required interface is provided
		InterfaceFlags required = child->pimpl()->requiredInterface();
		InterfaceFlags actual = child->pimpl()->interfaceFlags();
		if ((required & actual) != required)
			errors.push_back(*child, "required interface is not satisfied");

		// recursively validate all children and accumulate errors
		ContainerBase* child_container = dynamic_cast<ContainerBase*>(child.get());
		if (!child_container) continue;  // only containers provide validateConnectivity()
		try { child_container->validateConnectivity(); } catch (InitStageException &e) { errors.append(e); }
	}

	if (errors) throw errors;
}

std::ostream& operator<<(std::ostream& os, const ContainerBase& container) {
	ContainerBase::StageCallback processor = [&os](const Stage& stage, int depth) -> bool {
		os << std::string(2*depth, ' ') << *stage.pimpl() << std::endl;
		return true;
	};
	container.traverseRecursively(processor);
	return os;
}


struct SolutionCollector {
	SolutionCollector(size_t max_depth) : max_depth(max_depth) {}

	void operator()(const SerialContainer::solution_container& trace, double cost) {
		// traced path should not extend past container boundaries
		assert(trace.size() <= max_depth);
		solutions.emplace_back(std::make_pair(trace, cost));
	}

	typedef std::list<std::pair<SerialContainer::solution_container, double>> SolutionCostPairs;
	SolutionCostPairs solutions;
	const size_t max_depth;
};

void updateStateCosts(const SerialContainer::solution_container &partial_solution_path,
                      const InterfaceState::Priority &prio) {
	for (const SolutionBase* solution : partial_solution_path) {
		// here it suffices to update the start state, because the end state is the start state
		// of the next solution (they are all connected)
		InterfaceState* state = const_cast<InterfaceState*>(solution->start());
		if (state->owner()) state->owner()->updatePriority(state, prio);
	}
	// finally update the end state of the last solution
	if (partial_solution_path.empty()) return;
	InterfaceState* state = const_cast<InterfaceState*>(partial_solution_path.back()->end());
	if (state->owner()) state->owner()->updatePriority(state, prio);
}

void SerialContainer::onNewSolution(const SolutionBase &current)
{
	auto impl = pimpl();
	const StagePrivate *creator = current.creator();
	auto& children = impl->children();

	if (current.isFailure()) {
		return;  // don't consider failures
	}

	// find number of stages before and after creator stage
	size_t num_before = 0, num_after = 0;
	for (auto it = children.begin(), end = children.end(); it != end; ++it, ++num_before)
		if ((*it)->pimpl() == creator)
			break;
	assert(num_before < children.size());  // creator should be one of our children
	num_after = children.size()-1 - num_before;

	SerialContainer::solution_container trace; trace.reserve(children.size());

	// find all incoming solution paths ending at current solution
	SolutionCollector incoming(num_before);
	traverse<Interface::BACKWARD>(current, std::ref(incoming), trace);

	// find all outgoing solution paths starting at current solution
	SolutionCollector outgoing(num_after);
	traverse<Interface::FORWARD>(current, std::ref(outgoing), trace);

	// collect (and sort) all solutions spanning from start to end of this container
	ordered<SerialSolution> sorted;
	SerialContainer::solution_container solution;
	solution.reserve(children.size());
	for (auto& in : incoming.solutions) {
		for (auto& out : outgoing.solutions) {
			InterfaceState::Priority prio(in.first.size() + 1 + out.first.size(),
			                              in.second + current.cost() + out.second);
			// found a complete solution path connecting start to end?
			if (prio.depth() == children.size()) {
				assert(solution.empty());
				// insert incoming solutions in reverse order
				solution.insert(solution.end(), in.first.rbegin(), in.first.rend());
				// insert current solution
				solution.push_back(&current);
				// insert outgoing solutions in normal order
				solution.insert(solution.end(), out.first.begin(), out.first.end());
				// store solution in sorted list
				sorted.insert(SerialSolution(impl, std::move(solution), prio.cost()));
			} else if (prio.depth() > 1) {
				// update state priorities along the whole partial solution path
				updateStateCosts(in.first, prio);
				updateStateCosts({&current}, prio);
				updateStateCosts(out.first, prio);
			}
		}
	}

	// store new solutions (in sorted)
	for (auto it = sorted.begin(), end = sorted.end(); it != end; ++it) {
		auto inserted = impl->solutions_.insert(std::move(*it));
		impl->liftSolution(*inserted, inserted->internalStart(), inserted->internalEnd());
		impl->newSolution(*inserted);
	}
}


SerialContainer::SerialContainer(SerialContainerPrivate *impl)
   : ContainerBase(impl)
{}
SerialContainer::SerialContainer(const std::string &name)
   : SerialContainer(new SerialContainerPrivate(this, name))
{}

void SerialContainer::reset()
{
	auto impl = pimpl();

	// clear queues
	impl->solutions_.clear();

	// recursively reset children
	ContainerBase::reset();
}

SerialContainerPrivate::SerialContainerPrivate(SerialContainer *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{}

// a serial container's required interface is derived from the required input interface
// of the first child and the required output interface of the last child
InterfaceFlags SerialContainerPrivate::requiredInterface() const
{
	if (children().empty())
		return UNKNOWN;
	return (children().front()->pimpl()->requiredInterface() & INPUT_IF_MASK)
	      | (children().back()->pimpl()->requiredInterface() & OUTPUT_IF_MASK);
}

// connect cur stage to its predecessor and successor by setting the push interface pointers
// return true if cur stage should be scheduled for a second sweep
bool SerialContainerPrivate::connect(container_type::const_iterator cur)
{
	StagePrivate* const cur_impl = **cur;
	InterfaceFlags required = cur_impl->requiredInterface();

	// get iterators to prev / next stage in sequence
	auto prev = cur; --prev;
	auto next = cur; ++next;

	// set push forward connection using next's starts
	if ((required == UNKNOWN || required & WRITES_NEXT_START)
	    && next != children().end()) // last child has not a next one
		cur_impl->setNextStarts((*next)->pimpl()->starts());

	// set push backward connection using prev's ends
	if ((required == UNKNOWN || required & WRITES_PREV_END)
	    && cur != children().begin())  // first child has not a previous one
		cur_impl->setPrevEnds((*prev)->pimpl()->ends());

	// schedule stage with unknown interface for 2nd sweep
	return required == UNKNOWN;
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
void SerialContainer::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	// reset pull interfaces
	auto impl = pimpl();
	impl->starts_.reset();
	impl->ends_.reset();

	ContainerBase::init(robot_model); // throws if there are no children

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
		if (impl->connect(cur));
		else { // reached a stage with known interface
			// 2nd sweep: prune interfaces from [first_unknown, cur)
			impl->pruneInterfaces(first_unknown, cur);
			// restart with first_unknown = ++cur
			first_unknown = cur; ++first_unknown;
		}
	}
	// prune stages [first_unknown, end())
	impl->pruneInterfaces(first_unknown, impl->children().end());

	// initialize this' pull interfaces if first/last child pulls
	if (const InterfacePtr& target = (*start)->pimpl()->starts())
		impl->starts_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, _1, std::cref(target), _2)));
	if (const InterfacePtr& target = (*last)->pimpl()->ends())
		impl->ends_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, _1, std::cref(target), _2)));
}

// called by parent asking for pruning of this' interface
void SerialContainerPrivate::pruneInterface(InterfaceFlags accepted) {
	if (children().empty()) return;

	// We only need to deal with the special case of the whole sequence to be pruned.
	if (accepted != PROPAGATE_BOTHWAYS && // will interface be restricted at all?
	    children().front()->pimpl()->interfaceFlags() == PROPAGATE_BOTHWAYS)  // still undecided?
	{
		pruneInterfaces(children().begin(), children().end(), accepted);

		// reset my pull interfaces, if first/last child don't pull anymore
		if (!children().front()->pimpl()->starts())
			starts_.reset();
		if (!children().back()->pimpl()->ends())
			ends_.reset();
	}
	if (interfaceFlags() == UNKNOWN)
		throw InitStageException(*me(), "failed to derive propagation direction");
}

// called by init() to prune interfaces for children in range [first, last)
// this function determines the feasible propagation directions
void SerialContainerPrivate::pruneInterfaces(container_type::const_iterator first,
                                             container_type::const_iterator end)
{
	if (first == end) return;  // nothing to do in this case

	// determine accepted interface from available push interfaces
	InterfaceFlags accepted;

	// if first stage ...
	if (first != children().begin()) {
		auto prev = first; --prev; // pointer to previous stage
		// ... pushes forward, we accept forward propagation
		if ((*prev)->pimpl()->requiredInterface() & WRITES_NEXT_START)
			accepted |= PROPAGATE_FORWARDS;
		// ... pulls backward, we accept backward propagation
		if ((*prev)->pimpl()->requiredInterface() & READS_END)
			accepted |= PROPAGATE_BACKWARDS;
	} // else: for first child we cannot determine the interface yet

	// if end stage ...
	if (end != children().end()) {
		// ... pushes backward, we accept backward propagation
		if ((*end)->pimpl()->requiredInterface() & WRITES_PREV_END)
			accepted |= PROPAGATE_BACKWARDS;
		// ... pulls forward, we accept forward propagation
		if ((*end)->pimpl()->requiredInterface() & READS_START)
			accepted |= PROPAGATE_FORWARDS;
	} // else: for last child we cannot determine the interface yet

	// nothing to do if:
	// - accepted == 0: interface still unknown
	// - accepted == PROPAGATE_FORWARDS | PROPAGATE_BACKWARDS: no change
	if (accepted != UNKNOWN && accepted != InterfaceFlags({PROPAGATE_FORWARDS, PROPAGATE_BACKWARDS}))
		pruneInterfaces(first, end, accepted);
}

// prune interface for children in range [first, last) to given direction
void SerialContainerPrivate::pruneInterfaces(container_type::const_iterator first,
                                             container_type::const_iterator end,
                                             InterfaceFlags accepted)
{
	// 1st sweep: remove push interfaces
	for (auto it = first; it != end; ++it) {
		StagePrivate* impl = (*it)->pimpl();
		// range should only contain stages with unknown required interface
		assert(impl->requiredInterface() == UNKNOWN);

		// remove push interfaces
		if (!(accepted & PROPAGATE_BACKWARDS))
			impl->setPrevEnds(InterfacePtr());

		if (!(accepted & PROPAGATE_FORWARDS))
			impl->setNextStarts(InterfacePtr());
	}
	// 2nd sweep: recursively prune children
	for (auto it = first; it != end; ++it) {
		StagePrivate* impl = (*it)->pimpl();
		impl->pruneInterface(accepted);
	}
}

void SerialContainer::validateConnectivity() const
{
	auto impl = pimpl();
	InitStageException errors;

	// check that input / output interface of first / last child matches this' resp. interface
	if (!impl->children().empty()) {
		const StagePrivate* start = impl->children().front()->pimpl();
		if ((start->interfaceFlags() & INPUT_IF_MASK) != (this->pimpl()->interfaceFlags() & INPUT_IF_MASK))
			errors.push_back(*this, "input interface of '" + start->name() + "' doesn't match mine");

		const StagePrivate* last = impl->children().back()->pimpl();
		if ((last->interfaceFlags() & OUTPUT_IF_MASK) != (this->pimpl()->interfaceFlags() & OUTPUT_IF_MASK))
			errors.push_back(*this, "output interface of '" + last->name() + "' doesn't match mine");
	}

	// validate connectivity of children amongst each other
	// ContainerBase::validateConnectivity() ensures that required push interfaces are present,
	// that is, neighbouring stages have a corresponding pull interface.
	// Here, it remains to check that - if a child requires a pull interface - it's indeed feeded.
	for (auto cur = impl->children().begin(), end = impl->children().end(); cur != end; ++cur) {
		const StagePrivate* const cur_impl = **cur;
		InterfaceFlags required = cur_impl->requiredInterface();

		// get iterators to prev / next stage in sequence
		auto prev = cur; --prev;
		auto next = cur; ++next;

		// start pull interface fed?
		if (cur != impl->children().begin() &&  // first child has not a previous one
		    (required & READS_START) && !(*prev)->pimpl()->nextStarts())
			errors.push_back(**cur, "end interface is not fed");

		// end pull interface fed?
		if (next != end && // last child has not a next one
		    (required & READS_END) && !(*next)->pimpl()->prevEnds())
			errors.push_back(**cur, "end interface is not fed");
	}

	// recursively validate children
	try { ContainerBase::validateConnectivity(); } catch (InitStageException& e) { errors.append(e); }

	if (errors) throw errors;
}

bool SerialContainer::canCompute() const
{
	size_t num_finished = 0;
	for(const auto& stage : pimpl()->children()) {
		if (!stage->pimpl()->canCompute())
			++num_finished;
	}
	return num_finished < pimpl()->children().size();
}

bool SerialContainer::compute()
{
	bool computed = false;
	for(const auto& stage : pimpl()->children()) {
		try {
			if(!stage->pimpl()->canCompute())
				continue;

			ROS_INFO("Computing stage '%s'", stage->name().c_str());
			bool success = stage->pimpl()->compute();
			computed = true;
			ROS_INFO("Stage '%s': %s", stage->name().c_str(), success ? "succeeded" : "failed");
		} catch (const Property::error &e) {
			stage->reportPropertyError(e);
		}
	}
	return computed;
}

size_t SerialContainer::numSolutions() const
{
	return pimpl()->solutions_.size();
}

void SerialContainer::processSolutions(const ContainerBase::SolutionProcessor &processor) const
{
	for(const SolutionBase& s : pimpl()->solutions_)
		if (!processor(s))
			break;
}

template <Interface::Direction dir>
void SerialContainer::traverse(const SolutionBase &start, const SolutionProcessor &cb,
                               solution_container &trace, double trace_cost)
{
	const InterfaceState::Solutions& solutions = start.trajectories<dir>();
	if (solutions.empty())  // if we reached the end, call the callback
		cb(trace, trace_cost);
	else for (SolutionBase* successor : solutions) {
		trace.push_back(successor);
		trace_cost += successor->cost();

		traverse<dir>(*successor, cb, trace, trace_cost);

		trace_cost -= successor->cost();
		trace.pop_back();
	}
}

void SerialSolution::fillMessage(moveit_task_constructor_msgs::Solution &msg,
                                 Introspection* introspection) const
{
	moveit_task_constructor_msgs::SubSolution sub_msg;
	sub_msg.id = introspection ? introspection->solutionId(*this) : 0;
	sub_msg.cost = this->cost();

	const Introspection *ci = introspection;
	sub_msg.stage_id = ci ? ci->stageId(this->creator()->me()) : 0;

	sub_msg.sub_solution_id.reserve(subsolutions_.size());
	if (introspection) {
		for (const SolutionBase* s : subsolutions_)
			sub_msg.sub_solution_id.push_back(introspection->solutionId(*s));
		msg.sub_solution.push_back(sub_msg);
	}

	msg.sub_trajectory.reserve(msg.sub_trajectory.size() + subsolutions_.size());
	for (const SolutionBase* s : subsolutions_)
		s->fillMessage(msg, introspection);
}


void WrappedSolution::fillMessage(moveit_task_constructor_msgs::Solution &solution,
                                  Introspection *introspection) const
{
	wrapped_->fillMessage(solution, introspection);
}

ParallelContainerBasePrivate::ParallelContainerBasePrivate(ParallelContainerBase *me, const std::string &name)
   : ContainerBasePrivate(me, name)
{
}

// A parallel container's required interface is derived from the required interfaces of all of its children.
// They must not conflict to each other. Otherwise an InitStageException is thrown.
InterfaceFlags ParallelContainerBasePrivate::requiredInterface() const
{
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

		bool current_is_propagating = (current == PROPAGATE_BOTHWAYS ||
		                               current == PROPAGATE_FORWARDS ||
		                               current == PROPAGATE_BACKWARDS);

		if (current_is_propagating && accumulated != CONNECT && accumulated != GENERATE)
			accumulated |= current;  // propagating is compatible to all except CONNECT and GENERATE
		else
			throw InitStageException(*me(), "child '" + stage->name() + "' has conflicting interface to previous children");
	}
	return accumulated;
}

void ParallelContainerBasePrivate::pruneInterface(InterfaceFlags accepted)
{
	// forward pruning to all children with UNKNOWN required interface
	for (const Stage::pointer& stage : children()) {
		if (stage->pimpl()->requiredInterface() == UNKNOWN)
			stage->pimpl()->pruneInterface(accepted);
	}
}

void ParallelContainerBasePrivate::onNewExternalState(Interface::Direction dir, Interface::iterator external, bool updated) {
	for (const Stage::pointer& stage : children())
		copyState(external, stage->pimpl()->pullInterface(dir), updated);
}


ParallelContainerBase::ParallelContainerBase(ParallelContainerBasePrivate *impl)
   : ContainerBase(impl)
{}
ParallelContainerBase::ParallelContainerBase(const std::string &name)
   : ParallelContainerBase(new ParallelContainerBasePrivate(this, name))
{}

void ParallelContainerBase::reset()
{
	// recursively reset children
	ContainerBase::reset();
	// clear buffers
	auto impl = pimpl();
	impl->solutions_.clear();
	impl->failures_.clear();
	impl->wrapped_solutions_.clear();
	impl->created_solutions_.clear();
	impl->states_.clear();
}

/* States received by the container need to be copied to all children's pull interfaces.
 * States generated by children can be directly forwarded into the container's push interfaces.
 */
void ParallelContainerBase::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	// recursively init children
	ContainerBase::init(robot_model);
	auto impl = pimpl();

	// determine the union of interfaces required by children
	// TODO: should we better use the least common interface?
	InterfaceFlags required = impl->requiredInterface();

	// initialize this' pull connections
	impl->starts().reset(required & READS_START
	                     ? new Interface(std::bind(&ParallelContainerBasePrivate::onNewExternalState,
	                                               impl, Interface::FORWARD, _1, _2))
	                     : nullptr);
	impl->ends().reset(required & READS_END
	                   ? new Interface(std::bind(&ParallelContainerBasePrivate::onNewExternalState,
	                                             impl, Interface::BACKWARD, _1, _2))
	                   : nullptr);

	// initialize push connections of children according to their demands
	for (const Stage::pointer& stage : impl->children()) {
		impl->setChildsPushForwardInterface(*stage);
		impl->setChildsPushBackwardInterface(*stage);
	}
}

void ParallelContainerBase::validateConnectivity() const
{
	InitStageException errors;
	auto impl = pimpl();
	InterfaceFlags my_interface = impl->interfaceFlags();
	InterfaceFlags children_interfaces;

	// check that input / output interfaces of all children are handled by my interface
	for (const auto& child : pimpl()->children()) {
		InterfaceFlags current = child->pimpl()->interfaceFlags();
		children_interfaces |= current;  // compute union of all children interfaces
		if ((current & my_interface) != current)
			errors.push_back(*this, "interface of child '" + child->name() + "' doesn't match mine");
	}
	// check that there is a child matching the expected push interfaces
	if ((my_interface & GENERATE) != (children_interfaces & GENERATE))
		errors.push_back(*this, "no child provides expected push interface");

	// recursively validate children
	try { ContainerBase::validateConnectivity(); } catch (InitStageException& e) { errors.append(e); }

	if (errors) throw errors;
}

size_t ParallelContainerBase::numSolutions() const
{
	return pimpl()->solutions_.size();
}

void ParallelContainerBase::processSolutions(const Stage::SolutionProcessor &processor) const
{
	for(const SolutionBase* s : pimpl()->solutions_)
		if (!processor(*s))
			break;
}

size_t ParallelContainerBase::numFailures() const
{
	return pimpl()->failures_.size();
}

void ParallelContainerBase::processFailures(const Stage::SolutionProcessor &processor) const
{
	for(const SolutionBase* f : pimpl()->failures_)
		if (!processor(*f))
			break;
}

void ParallelContainerBase::onNewSolution(const SolutionBase& s)
{
	liftSolution(&s);
}

void ParallelContainerBase::liftSolution(const SolutionBase* solution, double cost)
{
	auto impl = pimpl();
	// create new WrappedSolution instance
	auto wit = impl->wrapped_solutions_.insert(impl->wrapped_solutions_.end(), WrappedSolution(impl, solution, cost));

	if (wit->isFailure()) {
		wit->setStartState(*solution->start());
		wit->setEndState(*solution->end());
		impl->failures_.push_back(&*wit);
	} else {
		impl->solutions_.insert(&*wit);
		impl->liftSolution(*wit, solution->start(), solution->end());
	}
	impl->newSolution(*wit);
}

void ParallelContainerBase::spawn(InterfaceState &&state, SubTrajectory&& t)
{
	auto impl = pimpl();
	assert(impl->prevEnds() && impl->nextStarts());

	t.setCreator(impl);
	// store newly created solution (otherwise it's lost)
	auto it = impl->created_solutions_.insert(impl->created_solutions_.end(), std::move(t));

	if (it->isFailure()) {
		// attach state (different for start / end) to trajectory
		auto state_it = impl->states_.insert(impl->states_.end(), InterfaceState(state));
		it->setStartState(*state_it);
		state_it = impl->states_.insert(impl->states_.end(), std::move(state));
		it->setEndState(*state_it);
		impl->failures_.push_back(&*it);
	} else {
		// directly spawn states in push interfaces
		impl->prevEnds()->add(InterfaceState(state), NULL, &*it);
		impl->nextStarts()->add(std::move(state), &*it, NULL);
		impl->solutions_.insert(&*it);
	}
	impl->newSolution(*it);
}


WrapperBasePrivate::WrapperBasePrivate(WrapperBase *me, const std::string &name)
   : ParallelContainerBasePrivate(me, name)
{}


WrapperBase::WrapperBase(const std::string &name, Stage::pointer &&child)
   : WrapperBase(new WrapperBasePrivate(this, name), std::move(child))
{}

WrapperBase::WrapperBase(WrapperBasePrivate *impl, Stage::pointer &&child)
   : ParallelContainerBase(impl)
{
	if (child) insert(std::move(child));
}

bool WrapperBase::insert(Stage::pointer &&stage, int before)
{
	// restrict num of children to one
	if (numChildren() > 0)
		return false;
	return ParallelContainerBase::insert(std::move(stage), before);
}

Stage* WrapperBase::wrapped()
{
	return pimpl()->children().empty() ? nullptr : pimpl()->children().front().get();
}

bool WrapperBase::canCompute() const
{
	return wrapped()->pimpl()->canCompute();
}

bool WrapperBase::compute()
{
	try {
		size_t num_before = numSolutions();
		wrapped()->pimpl()->compute();
		return numSolutions() > num_before;
	} catch (const Property::error &e) {
		wrapped()->reportPropertyError(e);
	}
	return false;
}


bool Alternatives::canCompute() const
{
	for (const auto& stage : pimpl()->children())
		if (stage->pimpl()->canCompute())
			return true;
	return false;
}

bool Alternatives::compute()
{
	bool success = false;
	for (const auto& stage : pimpl()->children()) {
		try {
			success |= stage->pimpl()->compute();
		} catch (const Property::error &e) {
			stage->reportPropertyError(e);
		}
	}
	return success;
}


void Fallbacks::reset()
{
	active_child_ = nullptr;
	ParallelContainerBase::reset();
}

void Fallbacks::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	ParallelContainerBase::init(robot_model);
	active_child_ = pimpl()->children().front().get();
}

bool Fallbacks::canCompute() const
{
	while (active_child_) {
		StagePrivate* child = active_child_->pimpl();
		if (child->canCompute()) return true;

		// active child failed, continue with next
		auto next = child->it(); ++next;
		active_child_ = next->get();
	}
	return false;
}

bool Fallbacks::compute()
{
	if (!active_child_)
		return false;

	try {
		return active_child_->pimpl()->compute();
	} catch (const Property::error &e) {
		active_child_->reportPropertyError(e);
	}
	return false;
}

} }
