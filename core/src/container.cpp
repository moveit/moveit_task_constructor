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
		position = from_end.base();
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

void ContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();
	auto& children = impl->children();

	Stage::init(scene);

	// we need to have some children to do the actual work
	if (children.empty()) {
		errors.push_back(*this, "no children");
		throw errors;
	}

	// recursively init all children
	for (auto& child : children) {
		try {
			child->init(scene);
		} catch (InitStageException &e) {
			errors.append(e);
		}
	}

	if (errors)
		throw errors;
}


struct SolutionCollector {
	SolutionCollector(size_t max_depth) : max_depth(max_depth) {}

	void operator()(const SerialContainer::solution_container& trace, double cost) {
		// traced path should not extend past container boundaries
		assert(trace.size() <= max_depth);
		solutions.emplace_back(std::make_pair(trace, cost));
	}

	std::list<std::pair<SerialContainer::solution_container, double>> solutions;
	const size_t max_depth;
};

void SerialContainer::onNewSolution(const SolutionBase &current)
{
	auto impl = pimpl();
	const StagePrivate *creator = current.creator();
	auto& children = impl->children();

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
				// update state costs
				const InterfaceState* start = (in.first.empty() ? current : *in.first.back()).start();
				start->owner()->updatePriority(*const_cast<InterfaceState*>(start), prio);
				const InterfaceState* end = (out.first.empty() ? current : *out.first.back()).end();
				end->owner()->updatePriority(*const_cast<InterfaceState*>(end), prio);
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

void SerialContainerPrivate::connect(StagePrivate* prev, StagePrivate* next) {
	prev->setNextStarts(next->starts());
	next->setPrevEnds(prev->ends());
}

void SerialContainer::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// if there are no children, there is nothing to connect
	if (!impl->children().empty()) {
		/*** connect children ***/
		// first stage sends backward to pending_backward_
		auto start = impl->children().begin();
		(*start)->pimpl()->setPrevEnds(impl->getPushBackwardInterface());

		// last stage sends forward to pending_forward_
		auto last = --impl->children().end();
		(*last)->pimpl()->setNextStarts(impl->getPushForwardInterface());

		auto cur = start;
		auto prev = cur; ++cur; // prev points to 1st, cur points to 2nd stage
		if (prev != last) {// we have more than one children
			auto next = cur; ++next; // next points to 3rd stage (or end)
			for (; cur != last; ++prev, ++cur, ++next) {
				impl->connect(**prev, **cur);
				impl->connect(**cur, **next);
			}
			// finally connect last == cur and prev stage
			impl->connect(**prev, **cur);
		}

		// recursively init + validate all children
		// this needs to be done *after* initializing the connections
		ContainerBase::init(scene);

		// initialize starts_ and ends_ interfaces
		if (const InterfacePtr& target = (*start)->pimpl()->starts())
			impl->starts_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, _1, std::cref(target), _2)));
		if (const InterfacePtr& target = (*last)->pimpl()->ends())
			impl->ends_.reset(new Interface(std::bind(&SerialContainerPrivate::copyState, impl, _1, std::cref(target), _2)));

		// validate connectivity of this
		if (!impl->nextStarts())
			errors.push_back(*this, "cannot sendForward()");
		if (!impl->prevEnds())
			errors.push_back(*this, "cannot sendBackward()");
	} else {
		errors.push_back(*this, "no children");
		// no children -> no reading
		impl->starts_.reset();
		impl->ends_.reset();
	}

	if (errors)
		throw errors;
}

bool SerialContainer::canCompute() const
{
	return !pimpl()->children().empty();
}

bool SerialContainer::compute()
{
	bool computed = false;
	for(const auto& stage : pimpl()->children()) {
		if(!stage->pimpl()->canCompute())
			continue;
		std::cout << "Computing stage '" << stage->name() << "':" << std::endl;
		bool success = stage->pimpl()->compute();
		computed = true;
		std::cout << (success ? "succeeded" : "failed") << std::endl;
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
void ParallelContainerBase::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	InitStageException errors;
	auto impl = pimpl();

	// initialize push connections of children
	InterfacePtr push_prev = impl->getPushBackwardInterface();
	InterfacePtr push_next = impl->getPushForwardInterface();
	for (const Stage::pointer& stage : impl->children()) {
		StagePrivate *child = stage->pimpl();
		child->setPrevEnds(push_prev);
		child->setNextStarts(push_next);
	}
	// recursively init + validate all children
	// this needs to be done *after* initializing push connections
	ContainerBase::init(scene);

	bool pulls[2];
	for (const Stage::pointer& stage : impl->children()) {
		StagePrivate *child = stage->pimpl();
		// is there any child reading from starts() or ends() ?
		pulls[Interface::START] |= bool(child->pullInterface(Interface::START));
		pulls[Interface::END] |= bool(child->pullInterface(Interface::END));
	}

	// initialize this' pull connections
	for (Interface::Direction dir : { Interface::START, Interface::END }) {
		if (pulls[dir])
			impl->pullInterface(dir).reset(new Interface(std::bind(&ParallelContainerBasePrivate::onNewExternalState, impl, dir, _1, _2)));
		else
			impl->pullInterface(dir).reset();
	}

	if (impl->children().empty())
		errors.push_back(*this, "no children");

	if (errors)
		throw errors;
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
	size_t num_before = numSolutions();
	wrapped()->pimpl()->compute();
	return numSolutions() > num_before;
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
	for (const auto& stage : pimpl()->children())
		success |= stage->pimpl()->compute();
	return success;
}


void Fallbacks::reset()
{
	active_child_ = nullptr;
	ParallelContainerBase::reset();
}

void Fallbacks::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	ParallelContainerBase::init(scene);
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
	if (!active_child_) return false;
	return active_child_->pimpl()->compute();
}

} }
