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
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <memory>
#include <iostream>
#include <algorithm>
#include <boost/range/adaptor/reversed.hpp>
#include <fmt/core.h>
#include <functional>

using namespace std::placeholders;
using namespace trajectory_processing;

namespace moveit {
namespace task_constructor {

// for debugging of how children interfaces evolve over time
__attribute__((unused))  // silent unused-function warning
static void
printChildrenInterfaces(const ContainerBasePrivate& container, bool success, const Stage& creator,
                        std::ostream& os = std::cerr) {
	static unsigned int id = 0;
	const unsigned int width = 10;  // indentation of name
	os << std::endl << (success ? '+' : '-') << ' ' << creator.name() << ' ';
	if (success)
		os << ++id << ' ';
	if (const auto conn = dynamic_cast<const ConnectingPrivate*>(creator.pimpl()))
		os << conn->pendingPairsPrinter();
	os << std::endl;

	for (const auto& child : container.children()) {
		auto cimpl = child->pimpl();
		os << std::setw(width) << std::left << child->name();
		if (!cimpl->starts() && !cimpl->ends())
			os << "↕ " << std::endl;
		if (cimpl->starts())
			os << "↓ " << *child->pimpl()->starts() << std::endl;
		if (cimpl->starts() && cimpl->ends())
			os << std::setw(width) << "  ";
		if (cimpl->ends())
			os << "↑ " << *child->pimpl()->ends() << std::endl;
	}
}

ContainerBasePrivate::ContainerBasePrivate(ContainerBase* me, const std::string& name)
  : StagePrivate(me, name)
  , required_interface_(UNKNOWN)
  , pending_backward_(new Interface)
  , pending_forward_(new Interface) {}

ContainerBasePrivate& ContainerBasePrivate::operator=(ContainerBasePrivate&& other) {
	assert(internal_external_.empty() && other.internal_external_.empty());

	// move StagePrivate members
	this->StagePrivate::operator=(std::move(other));

	// swapping of container members needed to maintain valid pending_* interfaces
	// and children (e.g. for TaskPrivate)
	required_interface_ = other.required_interface_;
	std::swap(pending_backward_, other.pending_backward_);
	std::swap(pending_forward_, other.pending_forward_);
	std::swap(children_, other.children_);

	// redirect all children's parent pointers to the new parent
	auto reparent_children = [](ContainerBasePrivate& self) {
		for (auto it = self.children_.begin(), end = self.children_.end(); it != end; ++it) {
			auto cimpl = (*it)->pimpl();
			cimpl->unparent();
			cimpl->setParent(static_cast<ContainerBase*>(self.me_));
			cimpl->setParentPosition(it);
		}
	};
	reparent_children(*this);
	reparent_children(other);
	return *this;
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
			return false;
		const ContainerBasePrivate* container = dynamic_cast<const ContainerBasePrivate*>(stage->pimpl());
		if (container)
			container->traverseStages(processor, cur_depth + 1, max_depth);
	}
	return true;
}

void ContainerBasePrivate::validateConnectivity() const {
	// recursively validate all children and accumulate errors
	for (const auto& child : children())
		child->pimpl()->validateConnectivity();
}

bool ContainerBasePrivate::canCompute() const {
	// call the method of the public interface
	return static_cast<ContainerBase*>(me_)->canCompute();
}

void ContainerBasePrivate::compute() {
	// call the method of the public interface
	static_cast<ContainerBase*>(me_)->compute();
}

template <Interface::Direction dir>
void ContainerBasePrivate::setStatus(const Stage* creator, const InterfaceState* source, const InterfaceState* target,
                                     InterfaceState::Status status) {
	if (status != InterfaceState::Status::ENABLED && creator) {
		if (const auto* conn = dynamic_cast<const Connecting*>(creator)) {
			auto cimpl = conn->pimpl();
			// if creator is a Connecting stage and target has enabled opposite states (other than source)
			if (cimpl->hasPendingOpposites<dir>(source, target))
				return;  // don't prune
		}
	}
	if (target->priority().status() == status)
		return;  // nothing changing

	// Skip disabling the state, if there are alternative enabled solutions
	if (status != InterfaceState::ENABLED) {
		auto solution_is_enabled = [](auto&& solution) {
			return state<opposite<dir>()>(*solution)->priority().enabled();
		};
		const auto& alternatives = trajectories<opposite<dir>()>(*target);
		auto alternative_path = std::find_if(alternatives.cbegin(), alternatives.cend(), solution_is_enabled);
		if (alternative_path != alternatives.cend())
			return;
	}

	// actually enable/disable the state
	const_cast<InterfaceState*>(target)->updateStatus(status);

	// if possible (i.e. if target has an external counterpart), escalate setStatus to external interface
	if (parent() && trajectories<dir>(*target).empty()) {
		// TODO: This was coded with SerialContainer in mind. Not sure, it works for ParallelContainers
		auto external{ internalToExternalMap().find(target) };
		if (external != internalToExternalMap().end()) {  // do we have an external state?
			// only escalate if there is no other *enabled* internal state connected to the same external one
			// all internal states linked to external
			auto internals{ externalToInternalMap().equal_range(external->get<EXTERNAL>()) };
			auto is_enabled = [](const auto& ext_int_pair) { return ext_int_pair.second->priority().enabled(); };
			auto other_path{ std::find_if(internals.first, internals.second, is_enabled) };
			if (other_path == internals.second)
				parent()->pimpl()->setStatus<dir>(nullptr, nullptr, external->get<EXTERNAL>(), status);
			return;
		}
	}

	// To break symmetry between both ends of a partial solution sequence that gets disabled,
	// we mark the first state with ARMED and all other states down the tree with PRUNED.
	// This allows us to re-enable the ARMED state, but not the PRUNED states,
	// when new states arrive in a Connecting stage.
	// For details, https://github.com/moveit/moveit_task_constructor/pull/309#issuecomment-974636202
	if (status == InterfaceState::Status::ARMED)
		status = InterfaceState::Status::PRUNED;  // only the first state is marked as ARMED

	// traverse solution tree
	for (const SolutionBase* successor : trajectories<dir>(*target))
		setStatus<dir>(successor->creator(), target, state<dir>(*successor), status);
}

// recursively update state priorities along solution path
template <Interface::Direction dir>
inline void updateStatePrios(const InterfaceState& s, const InterfaceState::Priority& prio) {
	InterfaceState::Priority priority(prio, s.priority().status());
	if (s.priority() == priority)
		return;
	const_cast<InterfaceState&>(s).updatePriority(priority);
	for (const SolutionBase* successor : trajectories<dir>(s))
		updateStatePrios<dir>(*state<dir>(*successor), prio);
}

void ContainerBasePrivate::onNewFailure(const Stage& child, const InterfaceState* from, const InterfaceState* to) {
	if (!static_cast<ContainerBase*>(me_)->pruning())
		return;

	RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Pruning"), fmt::format("'{}' generated a failure", child.name()));
	switch (child.pimpl()->interfaceFlags()) {
		case GENERATE:
			// just ignore: the pair of (new) states isn't known to us anyway
			// TODO: If child is a container, from and to might have associated solutions already!
			break;

		case PROPAGATE_FORWARDS:  // mark from as failed (backwards)
			setStatus<Interface::BACKWARD>(nullptr, nullptr, from, InterfaceState::Status::PRUNED);
			break;
		case PROPAGATE_BACKWARDS:  // mark to as failed (forwards)
			setStatus<Interface::FORWARD>(nullptr, nullptr, to, InterfaceState::Status::PRUNED);
			break;

		case CONNECT:
			setStatus<Interface::BACKWARD>(&child, to, from, InterfaceState::Status::ARMED);
			setStatus<Interface::FORWARD>(&child, from, to, InterfaceState::Status::ARMED);
			break;
	}
	// printChildrenInterfaces(*this, false, child);
}

template <Interface::Direction dir>
void ContainerBasePrivate::copyState(Interface::iterator external, const InterfacePtr& target,
                                     Interface::UpdateFlags updated) {
	if (updated) {
		auto prio = external->priority();
		auto internals = externalToInternalMap().equal_range(&*external);

		if (updated.testFlag(Interface::Update::STATUS)) {  // propagate external status updates to internal copies
			for (auto& i = internals.first; i != internals.second; ++i)
				setStatus<dir>(nullptr, nullptr, i->second, prio.status());
		} else if (updated.testFlag(Interface::Update::PRIORITY)) {
			for (auto& i = internals.first; i != internals.second; ++i)
				updateStatePrios<opposite<dir>()>(*i->second, prio);
		} else
			assert(false);  // Expecting either STATUS or PRIORITY updates, not both!
		return;
	}
	// create a clone of external state within target interface (child's starts() or ends())
	auto internal = states_.insert(states_.end(), InterfaceState(*external));
	target->add(*internal);
	// and remember the mapping between them
	internalToExternalMap().insert(std::make_pair(&*internal, &*external));
}

void ContainerBasePrivate::copyState(Interface::Direction dir, Interface::iterator external, const InterfacePtr& target,
                                     Interface::UpdateFlags updated) {
	if (dir == Interface::FORWARD)
		copyState<Interface::FORWARD>(external, target, updated);
	else
		copyState<Interface::BACKWARD>(external, target, updated);
}

void ContainerBasePrivate::liftSolution(const SolutionBasePtr& solution, const InterfaceState* internal_from,
                                        const InterfaceState* internal_to) {
	computeCost(*internal_from, *internal_to, *solution);

	// map internal to external states
	auto find_or_create_external = [this](const InterfaceState* internal, bool& created) -> InterfaceState* {
		auto it = internalToExternalMap().find(internal);
		if (it != internalToExternalMap().end())
			return const_cast<InterfaceState*>(it->second);

		InterfaceState* external = &*states_.insert(states_.end(), InterfaceState(*internal));
		internalToExternalMap().insert(std::make_pair(internal, external));
		created = true;
		return external;
	};
	bool created_from = false;
	bool created_to = false;
	InterfaceState* external_from = find_or_create_external(internal_from, created_from);
	InterfaceState* external_to = find_or_create_external(internal_to, created_to);

	if (!storeSolution(solution, external_from, external_to))
		return;

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

ContainerBase::ContainerBase(ContainerBasePrivate* impl) : Stage(impl) {
	auto& p = properties();
	p.declare<bool>("pruning", std::string("enable pruning?")).configureInitFrom(Stage::PARENT, "pruning");
}

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

Stage* ContainerBase::operator[](int index) const {
	auto impl = pimpl();
	auto it = impl->childByIndex(index, false);
	return it != impl->children().end() ? it->get() : nullptr;
}

bool ContainerBase::traverseChildren(const ContainerBase::StageCallback& processor) const {
	return pimpl()->traverseStages(processor, 0, 1);
}
bool ContainerBase::traverseRecursively(const ContainerBase::StageCallback& processor) const {
	if (!processor(*this, 0))
		return false;
	return pimpl()->traverseStages(processor, 1, UINT_MAX);
}

void ContainerBase::add(Stage::pointer&& stage) {
	insert(std::move(stage), -1);
}

void ContainerBase::insert(Stage::pointer&& stage, int before) {
	if (!stage)
		throw std::runtime_error(name() + ": received invalid stage pointer");

	StagePrivate* impl = stage->pimpl();
	impl->setParent(this);
	ContainerBasePrivate::const_iterator where = pimpl()->childByIndex(before, true);
	ContainerBasePrivate::iterator it = pimpl()->children_.insert(where, std::move(stage));
	impl->setParentPosition(it);
}

Stage::pointer ContainerBasePrivate::remove(ContainerBasePrivate::const_iterator pos) {
	if (pos == children_.end())
		return Stage::pointer();

	(*pos)->pimpl()->unparent();
	Stage::pointer result = std::move(*children_.erase(pos, pos));  // stage from non-const iterator to pos
	children_.erase(pos);  // actually erase stage
	return result;
}

Stage::pointer ContainerBase::remove(int pos) {
	return pimpl()->remove(pimpl()->childByIndex(pos, false));
}

Stage::pointer ContainerBase::remove(Stage* child) {
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
	impl->internalToExternalMap().clear();

	// interfaces depend on children which might change
	impl->required_interface_ = UNKNOWN;
	impl->starts_.reset();
	impl->ends_.reset();

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

void ContainerBase::explainFailure(std::ostream& os) const {
	for (const auto& stage : pimpl()->children()) {
		if (!stage->solutions().empty())
			continue;  // skip deeper traversal, this stage produced solutions
		if (stage->numFailures()) {
			os << stage->name() << " (0/" << stage->numFailures() << ")";
			stage->explainFailure(os);
			os << std::endl;
			break;
		}
		stage->explainFailure(os);  // recursively process children
	}
}

std::ostream& operator<<(std::ostream& os, const ContainerBase& container) {
	ContainerBase::StageCallback processor = [&os](const Stage& stage, unsigned int depth) -> bool {
		os << std::string(2 * depth, ' ') << *stage.pimpl() << std::endl;
		return true;
	};
	container.traverseRecursively(processor);
	return os;
}

/** Collect all partial solution sequences originating from start into given direction */
template <Interface::Direction dir>
struct SolutionCollector
{
	SolutionCollector(size_t max_depth, const SolutionBase& start) : max_depth(max_depth) {
		trace.reserve(max_depth);
		traverse(start, InterfaceState::Priority(0, 0.0));
		assert(trace.empty());
	}

	void traverse(const SolutionBase& start, const InterfaceState::Priority& prio) {
		const InterfaceState::Solutions& next = trajectories<dir>(*state<dir>(start));
		if (next.empty()) {  // when reaching the end, add the trace to solutions
			assert(prio.depth() == trace.size());
			assert(prio.depth() <= max_depth);
			solutions.emplace_back(std::make_pair(trace, prio));
		} else {
			for (SolutionBase* successor : next) {
				if (successor->isFailure())
					continue;  // skip failures
				trace.push_back(successor);
				traverse(*successor, prio + InterfaceState::Priority(1, successor->cost()));
				trace.pop_back();
			}
		}
	}

	using SolutionCostPairs = std::list<std::pair<SolutionSequence::container_type, InterfaceState::Priority>>;
	SolutionCostPairs solutions;
	const size_t max_depth;
	SolutionSequence::container_type trace;
};

void SerialContainer::onNewSolution(const SolutionBase& current) {
	RCLCPP_DEBUG_STREAM(rclcpp::get_logger("SerialContainer"), fmt::format("'{}' received solution of child stage '{}'",
	                                                                       this->name(), current.creator()->name()));

	// failures should never trigger this callback
	assert(!current.isFailure());

	auto impl = pimpl();
	const Stage* creator = current.creator();
	auto& children = impl->children();

	// find number of stages before and after creator stage
	size_t num_before = 0, num_after = 0;
	for (auto it = children.begin(), end = children.end(); it != end; ++it, ++num_before)
		if (&(**it) == creator)
			break;
	assert(num_before < children.size());  // creator should be one of our children
	num_after = children.size() - 1 - num_before;

	// find all incoming and outgoing solution paths originating from current solution
	SolutionCollector<Interface::BACKWARD> incoming(num_before, current);
	SolutionCollector<Interface::FORWARD> outgoing(num_after, current);

	// collect (and sort) all solutions spanning from start to end of this container
	ordered<SolutionSequencePtr> sorted;
	for (auto& in : incoming.solutions) {
		for (auto& out : outgoing.solutions) {
			InterfaceState::Priority prio = in.second + InterfaceState::Priority(1u, current.cost()) + out.second;
			assert(prio.enabled());
			// found a complete solution path connecting start to end?
			if (prio.depth() == children.size()) {
				SolutionSequence::container_type solution;
				solution.reserve(children.size());
				// insert incoming solutions in reverse order
				solution.insert(solution.end(), in.first.rbegin(), in.first.rend());
				// insert current solution
				solution.push_back(&current);
				// insert outgoing solutions in normal order
				solution.insert(solution.end(), out.first.begin(), out.first.end());
				// store solution in sorted list
				sorted.insert(std::make_shared<SolutionSequence>(std::move(solution), prio.cost(), this));
			}
			if (prio.depth() > 1) {
				// update state priorities along the whole partial solution path
				updateStatePrios<Interface::BACKWARD>(*current.start(), prio);
				updateStatePrios<Interface::FORWARD>(*current.end(), prio);
			}
		}
	}
	// printChildrenInterfaces(*this->pimpl(), true, *current.creator());

	// finally, store + announce new solutions to external interface
	for (const auto& solution : sorted)
		impl->liftSolution(solution, solution->internalStart(), solution->internalEnd());
}

SerialContainer::SerialContainer(SerialContainerPrivate* impl) : ContainerBase(impl) {}
SerialContainer::SerialContainer(const std::string& name) : SerialContainer(new SerialContainerPrivate(this, name)) {}

SerialContainerPrivate::SerialContainerPrivate(SerialContainer* me, const std::string& name)
  : ContainerBasePrivate(me, name) {}

void SerialContainerPrivate::connect(StagePrivate& stage1, StagePrivate& stage2) {
	InterfaceFlags flags1 = stage1.requiredInterface();
	InterfaceFlags flags2 = stage2.requiredInterface();

	if ((flags1 & WRITES_NEXT_START) && (flags2 & READS_START))
		stage1.setNextStarts(stage2.starts());
	else if ((flags1 & READS_END) && (flags2 & WRITES_PREV_END))
		stage2.setPrevEnds(stage1.ends());
	else {
		throw InitStageException(
		    *me(), fmt::format("cannot connect end interface of '{}' ({}) to start interface of '{}' ({})",  //
		                       stage1.name(), flowSymbol<END_IF_MASK>(flags1),  //
		                       stage2.name(), flowSymbol<START_IF_MASK>(flags2)));
	}
}

template <unsigned int mask>
void SerialContainerPrivate::validateInterface(const StagePrivate& child, InterfaceFlags required) const {
	required = required & mask;
	if (required == UNKNOWN)
		return;  // cannot yet validate
	InterfaceFlags child_interface = child.interfaceFlags() & mask;
	if (required != child_interface)
		throw InitStageException(*me_, fmt::format("{0} interface ({2}) of '{1}' does not match mine ({3})",
		                                           (mask == START_IF_MASK ? "start" : "end"), child.name(),
		                                           flowSymbol<mask>(child_interface), flowSymbol<mask>(required)));
}

// called by parent asking for pruning of this' interface
void SerialContainerPrivate::resolveInterface(InterfaceFlags expected) {
	// we need to have some children to do the actual work
	if (children().empty())
		throw InitStageException(*me(), "no children");

	if (!(expected & START_IF_MASK))
		throw InitStageException(*me(), "unknown start interface");

	Stage& first = *children().front();
	Stage& last = *children().back();

	InitStageException exceptions;

	try {  // FIRST child
		first.pimpl()->resolveInterface(expected & START_IF_MASK);
		// connect first child's (start) push interface
		setChildsPushBackwardInterface(first.pimpl());
		// validate that first child's and this container's start interfaces match
		validateInterface<START_IF_MASK>(*first.pimpl(), expected);
		// connect first child's (start) pull interface
		if (const InterfacePtr& target = first.pimpl()->starts())
			starts_ = std::make_shared<Interface>([this, target](Interface::iterator it, Interface::UpdateFlags updated) {
				this->copyState<Interface::FORWARD>(it, target, updated);
			});
	} catch (InitStageException& e) {
		exceptions.append(e);
	}

	// process all children and connect them
	for (auto it = ++children().begin(), previous_it = children().begin(); it != children().end(); ++it, ++previous_it) {
		try {
			StagePrivate* child_impl = (**it).pimpl();
			StagePrivate* previous_impl = (**previous_it).pimpl();
			child_impl->resolveInterface(invert(previous_impl->requiredInterface()) & START_IF_MASK);
			child_impl = (**it).pimpl();  // re-assign as pimpl_ pointer of a Fallback container will change!
			connect(*previous_impl, *child_impl);
		} catch (InitStageException& e) {
			exceptions.append(e);
		}
	}

	try {  // connect last child's (end) push interface
		setChildsPushForwardInterface(last.pimpl());
		// validate that last child's and this container's end interfaces match
		validateInterface<END_IF_MASK>(*last.pimpl(), expected);
		// connect last child's (end) pull interface
		if (const InterfacePtr& target = last.pimpl()->ends())
			ends_ = std::make_shared<Interface>([this, target](Interface::iterator it, Interface::UpdateFlags updated) {
				this->copyState<Interface::BACKWARD>(it, target, updated);
			});
	} catch (InitStageException& e) {
		exceptions.append(e);
	}

	required_interface_ = (first.pimpl()->interfaceFlags() & START_IF_MASK) |  //
	                      (last.pimpl()->interfaceFlags() & END_IF_MASK);

	if (exceptions)
		throw exceptions;
}

void SerialContainerPrivate::validateConnectivity() const {
	ContainerBasePrivate::validateConnectivity();

	InterfaceFlags mine = interfaceFlags();
	// check that input/output interface of first/last child matches this' resp. interface
	validateInterface<START_IF_MASK>(*children().front()->pimpl(), mine);
	validateInterface<END_IF_MASK>(*children().back()->pimpl(), mine);

	// validate connectivity of children between each other
	// ContainerBasePrivate::validateConnectivity() ensures that required push interfaces are present,
	// that is, neighbouring stages have a corresponding pull interface.
	// Here, it remains to check that - if a child has a pull interface - it's indeed feeded.
	for (auto cur = children().begin(), end = children().end(); cur != end; ++cur) {
		const StagePrivate* const cur_impl = **cur;
		InterfaceFlags required = cur_impl->interfaceFlags();

		// get iterators to prev/next stage in sequence
		auto prev = cur;
		--prev;
		auto next = cur;
		++next;

		// start pull interface fed?
		if (cur != children().begin() &&  // first child has not a previous one
		    (required & READS_START) && !(*prev)->pimpl()->nextStarts())
			throw InitStageException(**cur, "start interface is not fed");

		// end pull interface fed?
		if (next != end &&  // last child has not a next one
		    (required & READS_END) && !(*next)->pimpl()->prevEnds())
			throw InitStageException(**cur, "end interface is not fed");
	}
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
		if (stage->pimpl()->canCompute())
			stage->pimpl()->runCompute();
	}
}

ParallelContainerBasePrivate::ParallelContainerBasePrivate(ParallelContainerBase* me, const std::string& name)
  : ContainerBasePrivate(me, name) {}

void ParallelContainerBasePrivate::resolveInterface(InterfaceFlags expected) {
	// we need to have some children to do the actual work
	if (children().empty())
		throw InitStageException(*me(), "no children");

	InitStageException exceptions;

	bool first = true;
	for (const Stage::pointer& child : children()) {
		try {
			auto child_impl = child->pimpl();
			child_impl->resolveInterface(expected);
			validateInterfaces(*child_impl, expected, first);
			// initialize push connections of children according to their demands
			setChildsPushBackwardInterface(child_impl);
			setChildsPushForwardInterface(child_impl);
			first = false;
		} catch (InitStageException& e) {
			exceptions.append(e);
			continue;
		}
	}

	if (exceptions)
		throw exceptions;

	required_interface_ = expected;

	initializeExternalInterfaces();
}

void ParallelContainerBasePrivate::initializeExternalInterfaces() {
	// States received by the container need to be copied to all children's pull interfaces.
	if (requiredInterface() & READS_START)
		starts() = std::make_shared<Interface>([this](Interface::iterator external, Interface::UpdateFlags updated) {
			this->propagateStateToAllChildren<Interface::FORWARD>(external, updated);
		});
	if (requiredInterface() & READS_END)
		ends() = std::make_shared<Interface>([this](Interface::iterator external, Interface::UpdateFlags updated) {
			this->propagateStateToAllChildren<Interface::BACKWARD>(external, updated);
		});
}

void ParallelContainerBasePrivate::validateInterfaces(const StagePrivate& child, InterfaceFlags& external,
                                                      bool first) const {
	const InterfaceFlags child_interface = child.requiredInterface();
	bool valid = true;
	for (InterfaceFlags mask : { START_IF_MASK, END_IF_MASK }) {
		if ((external & mask) == UNKNOWN)
			external |= child_interface & mask;

		valid = valid & ((external & mask) == (child_interface & mask));
	}

	if (!valid)
		throw InitStageException(
		    *me_, fmt::format("interface of '{0}' ({2} {3}) does not match {1} ({4} {5}).", child.name(),
		                      (first ? "external one" : "other children's"),  //
		                      flowSymbol<START_IF_MASK>(child_interface), flowSymbol<END_IF_MASK>(child_interface),
		                      flowSymbol<START_IF_MASK>(external), flowSymbol<END_IF_MASK>(external)));
}

void ParallelContainerBasePrivate::validateConnectivity() const {
	InterfaceFlags my_interface = interfaceFlags();

	// check that input/output interfaces of all children are handled by my interface
	for (const auto& child : children())
		validateInterfaces(*child->pimpl(), my_interface);

	ContainerBasePrivate::validateConnectivity();
}

template <Interface::Direction dir>
void ParallelContainerBasePrivate::propagateStateToAllChildren(Interface::iterator external,
                                                               Interface::UpdateFlags updated) {
	for (const Stage::pointer& stage : children())
		copyState<dir>(external, stage->pimpl()->pullInterface<dir>(), updated);
}

ParallelContainerBase::ParallelContainerBase(ParallelContainerBasePrivate* impl) : ContainerBase(impl) {}
ParallelContainerBase::ParallelContainerBase(const std::string& name)
  : ParallelContainerBase(new ParallelContainerBasePrivate(this, name)) {}

void ParallelContainerBase::liftSolution(const SolutionBase& solution, double cost, std::string comment) {
	pimpl()->liftSolution(std::make_shared<WrappedSolution>(this, &solution, cost, std::move(comment)), solution.start(),
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

void WrapperBase::insert(Stage::pointer&& stage, int before) {
	// restrict num of children to one
	if (numChildren() > 0)
		throw std::runtime_error(name() + ": Wrapper only allows a single child");
	return ParallelContainerBase::insert(std::move(stage), before);
}

Stage* WrapperBase::wrapped() {
	return pimpl()->children().empty() ? nullptr : pimpl()->children().front().get();
}

bool WrapperBase::canCompute() const {
	return wrapped()->pimpl()->canCompute();
}

void WrapperBase::compute() {
	wrapped()->pimpl()->runCompute();
}

bool Alternatives::canCompute() const {
	for (const auto& stage : pimpl()->children())
		if (stage->pimpl()->canCompute())
			return true;
	return false;
}

void Alternatives::compute() {
	for (const auto& stage : pimpl()->children()) {
		stage->pimpl()->runCompute();
	}
}

void Alternatives::onNewSolution(const SolutionBase& s) {
	liftSolution(s);
}

Fallbacks::Fallbacks(const std::string& name) : Fallbacks(new FallbacksPrivate(this, name)) {}

Fallbacks::Fallbacks(FallbacksPrivate* impl) : ParallelContainerBase(impl) {}

void Fallbacks::reset() {
	ParallelContainerBase::reset();
	pimpl()->reset();
}

void Fallbacks::init(const moveit::core::RobotModelConstPtr& robot_model) {
	ParallelContainerBase::init(robot_model);
	pimpl()->reset();
}

void Fallbacks::onNewSolution(const SolutionBase& s) {
	pimpl()->onNewSolution(s);
}

inline void Fallbacks::replaceImpl() {
	FallbacksPrivate* impl = pimpl();
	if (pimpl()->interfaceFlags() == pimpl()->requiredInterface())
		return;
	switch (pimpl()->requiredInterface()) {
		case GENERATE:
			impl = new FallbacksPrivateGenerator(std::move(*impl));
			break;
		case PROPAGATE_FORWARDS:
		case PROPAGATE_BACKWARDS:
			impl = new FallbacksPrivatePropagator(std::move(*impl));
			break;
		case CONNECT:
			// For now, we only support Connecting children
			for (const auto& child : impl->children())
				if (!dynamic_cast<Connecting*>(child.get()))
					throw std::runtime_error("CONNECT-like interface is only supported for Connecting children");
			impl = new FallbacksPrivateConnect(std::move(*impl));
			break;
	}
	delete pimpl_;
	pimpl_ = impl;
}

FallbacksPrivate::FallbacksPrivate(Fallbacks* me, const std::string& name) : ParallelContainerBasePrivate(me, name) {}

FallbacksPrivate::FallbacksPrivate(FallbacksPrivate&& other)
  : ParallelContainerBasePrivate(static_cast<Fallbacks*>(other.me()), "") {
	// move contents of other
	this->ParallelContainerBasePrivate::operator=(std::move(other));
}

void FallbacksPrivate::initializeExternalInterfaces() {
	// Here we know the final interface of the container (and all its children)
	// Thus replace, this pimpl() with a new interface-specific one:
	static_cast<Fallbacks*>(me())->replaceImpl();
}

void FallbacksPrivate::onNewSolution(const SolutionBase& s) {
	// printChildrenInterfaces(*this, true, *s.creator());
	static_cast<Fallbacks*>(me())->liftSolution(s);
}

void FallbacksPrivate::onNewFailure(const Stage& child, const InterfaceState* /*from*/, const InterfaceState* /*to*/) {
	// This override is deliberately empty.
	// The method prunes solution paths when a child failed to find a valid solution for it,
	// but in Fallbacks the next child might still yield a successful solution
	// Thus pruning must only occur once the last child is exhausted (inside computePropagate)
	// printChildrenInterfaces(*this, false, child);
	(void)child;
}

void FallbacksPrivateCommon::reset() {
	current_ = children().begin();
}

bool FallbacksPrivateCommon::canCompute() const {
	while (current_ != children().end() &&  // not completely exhausted
	       !(*current_)->pimpl()->canCompute())  // but current child cannot compute
		return const_cast<FallbacksPrivateCommon*>(this)->nextJob();  // advance to next job

	// return value: current child is well defined and thus can compute?
	return current_ != children().end();
}

void FallbacksPrivateCommon::compute() {
	(*current_)->pimpl()->runCompute();
}

inline void FallbacksPrivateCommon::nextChild() {
	if (std::next(current_) != children().end())
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Fallbacks"),
		                    fmt::format("Child '{}' failed, trying next one.", (*current_)->name()));
	++current_;  // advance to next child
}

FallbacksPrivateGenerator::FallbacksPrivateGenerator(FallbacksPrivate&& old) : FallbacksPrivateCommon(std::move(old)) {
	FallbacksPrivateCommon::reset();
}

bool FallbacksPrivateGenerator::nextJob() {
	assert(current_ != children().end() && !(*current_)->pimpl()->canCompute());

	// don't advance to next child when we already produced solutions
	if (!solutions_.empty()) {
		current_ = children().end();  // indicate that we are exhausted
		return false;
	}

	do {
		nextChild();
	} while (current_ != children().end() && !(*current_)->pimpl()->canCompute());

	// return value shall indicate current_->canCompute()
	return current_ != children().end();
}

FallbacksPrivatePropagator::FallbacksPrivatePropagator(FallbacksPrivate&& old)
  : FallbacksPrivateCommon(std::move(old)) {
	switch (requiredInterface()) {
		case PROPAGATE_FORWARDS:
			dir_ = Interface::FORWARD;
			starts() = std::make_shared<Interface>();
			break;
		case PROPAGATE_BACKWARDS:
			dir_ = Interface::BACKWARD;
			ends() = std::make_shared<Interface>();
			break;
		default:
			assert(false);
	}
	FallbacksPrivatePropagator::reset();
}

void FallbacksPrivatePropagator::reset() {
	FallbacksPrivateCommon::reset();
	job_ = pullInterface(dir_)->end();  // indicate fresh start
	job_has_solutions_ = false;
}

void FallbacksPrivatePropagator::onNewSolution(const SolutionBase& s) {
	job_has_solutions_ = true;
	FallbacksPrivateCommon::onNewSolution(s);
}

bool FallbacksPrivatePropagator::nextJob() {
	assert(current_ != children().end() && !(*current_)->pimpl()->canCompute());
	const auto jobs = pullInterface(dir_);

	if (job_ != jobs->end()) {  // current job exists, but is exhausted on current child
		if (!job_has_solutions_)  // job didn't produce solutions -> feed to next child
			nextChild();
		else
			current_ = children().end();  // indicate that this job is exhausted on all children
	}
	job_has_solutions_ = false;

	if (current_ == children().end()) {  // all children processed the job_
		if (job_ != jobs->end()) {
			jobs->remove(job_);  // we don't need the job in our interface list anymore
			job_ = jobs->end();  // indicate that we need to fetch a new job
		}
		current_ = children().begin();  // start next job with first child again
	}

	// pick next job if needed and possible
	if (job_ == jobs->end()) {  // need to pick next job
		if (!jobs->empty() && jobs->front()->priority().enabled())
			job_ = jobs->begin();
		else
			return false;  // no more jobs available
	}

	// When arriving here, we have a valid job_ and a current_ child to feed it. Let's do that.
	copyState(dir_, job_, (*current_)->pimpl()->pullInterface(dir_), Interface::UpdateFlags());
	return true;
}

FallbacksPrivateConnect::FallbacksPrivateConnect(FallbacksPrivate&& old) : FallbacksPrivate(std::move(old)) {
	starts_ = std::make_shared<Interface>(std::bind(&FallbacksPrivateConnect::propagateStateUpdate<Interface::FORWARD>,
	                                                this, std::placeholders::_1, std::placeholders::_2));
	ends_ = std::make_shared<Interface>(std::bind(&FallbacksPrivateConnect::propagateStateUpdate<Interface::BACKWARD>,
	                                              this, std::placeholders::_1, std::placeholders::_2));

	FallbacksPrivateConnect::reset();
}

void FallbacksPrivateConnect::reset() {
	active_ = children().end();
}

template <Interface::Direction dir>
void FallbacksPrivateConnect::propagateStateUpdate(Interface::iterator external, Interface::UpdateFlags updated) {
	copyState<dir>(external, children().front()->pimpl()->pullInterface(dir), updated);
	// As we use the Interface* from the first child for all children (we just populate their pending lists)
	// there is no need to explicitly propagate state updates to other children.
}

bool FallbacksPrivateConnect::canCompute() const {
	for (auto it = children().begin(), end = children().end(); it != end; ++it)
		if ((*it)->pimpl()->canCompute()) {
			active_ = it;
			return true;
		}
	active_ = children().end();
	return false;
}

void FallbacksPrivateConnect::compute() {
	// Alternatively, we could also compute() all children that canCompute()
	assert(active_ != children().end());
	(*active_)->pimpl()->runCompute();
}

void FallbacksPrivateConnect::onNewFailure(const Stage& child, const InterfaceState* from, const InterfaceState* to) {
	// expect failure to be reported from active child
	assert(active_ != children().end() && active_->get() == &child);
	(void)child;
	// ... thus we can use std::next(active_) to find the next child
	auto next = std::next(active_);

	// NOLINTNEXTLINE(readability-identifier-naming)
	auto findIteratorFor = [](const InterfaceState* state, const Interface& interface) {
		auto it = std::find(interface.begin(), interface.end(), state);
		assert(it != interface.end());
		return it;
	};

	if (next != children().end()) {  // pass job to next child
		auto next_con = static_cast<ConnectingPrivate*>(const_cast<StagePrivate*>((*next)->pimpl()));
		auto first_con = static_cast<const ConnectingPrivate*>(children().front()->pimpl());
		auto from_it = findIteratorFor(from, *first_con->starts());
		auto to_it = findIteratorFor(to, *first_con->ends());
		next_con->pending.insert(std::make_pair(from_it, to_it));
	} else  // or report failure to parent
		parent()->pimpl()->onNewFailure(*me(), from, to);
}

MergerPrivate::MergerPrivate(Merger* me, const std::string& name) : ParallelContainerBasePrivate(me, name) {}

void MergerPrivate::resolveInterface(InterfaceFlags expected) {
	ParallelContainerBasePrivate::resolveInterface(expected);
	switch (requiredInterface()) {
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
}

Merger::Merger(const std::string& name) : Merger(new MergerPrivate(this, name)) {
	properties().declare<TimeParameterizationPtr>("time_parameterization",
	                                              std::make_shared<TimeOptimalTrajectoryGeneration>());
}

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
		stage->pimpl()->runCompute();
	}
}

void Merger::onNewSolution(const SolutionBase& s) {
	if (s.isFailure())  // ignore failure solutions
		return;

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
	if (!trajectory || !trajectory->trajectory()) {
		RCLCPP_ERROR(rclcpp::get_logger("Merger"), "Only simple, valid trajectories are supported");
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
	const InterfaceState* external_source_state = &*source_it->second;

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
	if (t.trajectory() && !t.trajectory()->empty())
		to->setCurrentState(t.trajectory()->getLastWayPoint());
	StagePrivate::sendForward(*from, InterfaceState(to), std::make_shared<SubTrajectory>(std::move(t)));
}

void MergerPrivate::sendBackward(SubTrajectory&& t, const InterfaceState* to) {
	// generate target state
	planning_scene::PlanningScenePtr from = to->scene()->diff();
	if (t.trajectory() && !t.trajectory()->empty())
		from->setCurrentState(t.trajectory()->getFirstWayPoint());
	StagePrivate::sendBackward(InterfaceState(from), *to, std::make_shared<SubTrajectory>(std::move(t)));
}

void MergerPrivate::onNewGeneratorSolution(const SolutionBase& /* s */) {
	// TODO: implement in similar fashion as onNewPropagateSolution(), but also merge start/end states
}

void MergerPrivate::mergeAnyCombination(const ChildSolutionMap& all_solutions, const SolutionBase& current,
                                        const planning_scene::PlanningSceneConstPtr& start_scene,
                                        const Spawner& spawner) {
	std::vector<size_t> indices;  // which solution index was considered last for i-th child?
	indices.reserve(children().size());

	ChildSolutionList sub_solutions;
	sub_solutions.reserve(children().size());

	// initialize vector of sub solutions
	for (const auto& pair : all_solutions) {
		// all children, except current solution's creator, start with zero index
		indices.push_back(pair.first != current.creator() ? 0 : pair.second.size() - 1);
		sub_solutions.push_back(pair.second[indices.back()]);
	}
	while (true) {
		merge(sub_solutions, start_scene, spawner);

		// compose next combination
		size_t child = 0;
		for (auto it = all_solutions.cbegin(), end = all_solutions.cend(); it != end; ++it, ++child) {
			if (it->first == current.creator())
				continue;  // skip current solution's child
			if (++indices[child] >= it->second.size()) {
				indices[child] = 0;  // start over with zero
				sub_solutions[child] = it->second[indices[child]];
				continue;  // and continue with next child
			}
			// otherwise, a new solution combination is available
			sub_solutions[child] = it->second[indices[child]];
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
	for (const auto& sub : sub_solutions)
		sub_trajectories.push_back(sub->trajectory());

	moveit::core::JointModelGroup* jmg = jmg_merged_.get();
	robot_trajectory::RobotTrajectoryPtr merged;
	try {
		auto timing = me_->properties().get<TimeParameterizationPtr>("time_parameterization");
		merged = task_constructor::merge(sub_trajectories, start_scene->getCurrentState(), jmg, *timing);
	} catch (const std::runtime_error& e) {
		SubTrajectory t;
		t.markAsFailure();
		t.setComment(e.what());
		spawner(std::move(t));
		return;
	}
	if (jmg_merged_.get() != jmg)
		jmg_merged_.reset(jmg);

	assert(merged);
	SubTrajectory t(merged);

	// check merged trajectory for collisions
	std::vector<std::size_t> invalid_index;
	if (!start_scene->isPathValid(*merged, "", true, &invalid_index)) {
		t.markAsFailure();
		std::ostringstream oss;
		oss << "Invalid waypoint(s): ";
		if (invalid_index.size() == merged->getWayPointCount())
			oss << "all";
		else
			for (size_t i : invalid_index)
				oss << i << ", ";
		t.setComment(oss.str());
	} else {
		// accumulate costs and markers
		double costs = 0.0;
		for (const auto& sub : sub_solutions) {
			costs += sub->cost();
			t.markers().insert(t.markers().end(), sub->markers().begin(), sub->markers().end());
		}
		t.setCost(costs);
	}
	spawner(std::move(t));
}
}  // namespace task_constructor
}  // namespace moveit
