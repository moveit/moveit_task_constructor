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

/* Authors: Michael Goerner, Robert Haschke */

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/introspection.h>
#include <moveit/planning_scene/planning_scene.h>

#include <ros/console.h>

#include <boost/format.hpp>

#include <iostream>
#include <iomanip>
#include <algorithm>
#include <utility>

namespace moveit {
namespace task_constructor {

template <>
const char* flowSymbol<START_IF_MASK>(InterfaceFlags f) {
	f = f & START_IF_MASK;
	if (f == READS_START)
		return "→";
	if (f == WRITES_PREV_END)
		return "←";
	if (f == UNKNOWN)
		return "?";
	return "↔";
}

template <>
const char* flowSymbol<END_IF_MASK>(InterfaceFlags f) {
	f = f & END_IF_MASK;
	if (f == READS_END)
		return "←";
	if (f == WRITES_NEXT_START)
		return "→";
	if (f == UNKNOWN)
		return "?";
	return "↔";
}

void InitStageException::push_back(const Stage& stage, const std::string& msg) {
	errors_.emplace_back(std::make_pair(&stage, msg));
}

void InitStageException::append(InitStageException& other) {
	errors_.splice(errors_.end(), other.errors_);
}

const char* InitStageException::what() const noexcept {
	static const char* msg = "Error initializing stage(s). ROS_ERROR_STREAM(e) for details.";
	return msg;
}

std::ostream& operator<<(std::ostream& os, const InitStageException& e) {
	os << "Error initializing stage" << (e.errors_.size() > 1 ? "s" : "") << ":" << std::endl;
	for (const auto& pair : e.errors_)
		os << pair.first->name() << ": " << pair.second << std::endl;
	return os;
}

StagePrivate::StagePrivate(Stage* me, const std::string& name)
  : me_(me), name_(name), total_compute_time_{}, parent_(nullptr), introspection_(nullptr) {}

InterfaceFlags StagePrivate::interfaceFlags() const {
	InterfaceFlags f;
	if (starts())
		f |= READS_START;
	if (ends())
		f |= READS_END;
	if (prevEnds())
		f |= WRITES_PREV_END;
	if (nextStarts())
		f |= WRITES_NEXT_START;
	return f;
}

void StagePrivate::validateConnectivity() const {
	// check that the required interface is provided
	InterfaceFlags required = requiredInterface();
	InterfaceFlags actual = interfaceFlags();
	if ((required & actual) != required) {
		boost::format desc("actual interface %1% %2% does not match required interface %3% %4%");
		desc % flowSymbol<START_IF_MASK>(actual) % flowSymbol<END_IF_MASK>(actual);
		desc % flowSymbol<START_IF_MASK>(required) % flowSymbol<END_IF_MASK>(required);
		throw InitStageException(*me(), desc.str());
	}
}

bool StagePrivate::storeSolution(const SolutionBasePtr& solution) {
	solution->setCreator(this);
	if (introspection_)
		introspection_->registerSolution(*solution);

	if (solution->isFailure()) {
		++num_failures_;
		if (!storeFailures())
			return false;  // drop solution
		failures_.push_back(solution);
	} else {
		solutions_.insert(solution);
	}
	return true;
}

void StagePrivate::sendForward(const InterfaceState& from, InterfaceState&& to, const SolutionBasePtr& solution) {
	assert(nextStarts());
	if (!storeSolution(solution))
		return;  // solution dropped
	me()->forwardProperties(from, to);

	auto to_it = states_.insert(states_.end(), std::move(to));

	solution->setStartState(from);
	solution->setEndState(*to_it);

	if (!solution->isFailure())
		nextStarts()->add(*to_it);

	newSolution(solution);
}

void StagePrivate::sendBackward(InterfaceState&& from, const InterfaceState& to, const SolutionBasePtr& solution) {
	assert(prevEnds());
	if (!storeSolution(solution))
		return;  // solution dropped
	me()->forwardProperties(to, from);

	auto from_it = states_.insert(states_.end(), std::move(from));

	solution->setStartState(*from_it);
	solution->setEndState(to);

	if (!solution->isFailure())
		prevEnds()->add(*from_it);

	newSolution(solution);
}

void StagePrivate::spawn(InterfaceState&& state, const SolutionBasePtr& solution) {
	assert(prevEnds() && nextStarts());
	if (!storeSolution(solution))
		return;  // solution dropped

	auto from = states_.insert(states_.end(), InterfaceState(state));  // copy
	auto to = states_.insert(states_.end(), std::move(state));

	solution->setStartState(*from);
	solution->setEndState(*to);

	if (!solution->isFailure()) {
		prevEnds()->add(*from);
		nextStarts()->add(*to);
	}

	newSolution(solution);
}

void StagePrivate::connect(const InterfaceState& from, const InterfaceState& to, const SolutionBasePtr& solution) {
	if (!storeSolution(solution))
		return;  // solution dropped

	solution->setStartState(from);
	solution->setEndState(to);

	newSolution(solution);
}

void StagePrivate::newSolution(const SolutionBasePtr& solution) {
	// call solution callbacks for both, valid solutions and failures
	for (const auto& cb : solution_cbs_)
		cb(*solution);

	if (parent() && !solution->isFailure())
		parent()->onNewSolution(*solution);
}

Stage::Stage(StagePrivate* impl) : pimpl_(impl) {
	assert(impl);
	auto& p = properties();
	p.declare<double>("timeout", "timeout per run (s)");
	p.declare<std::string>("marker_ns", name(), "marker namespace");

	p.declare<std::set<std::string>>("forwarded_properties", std::set<std::string>(),
	                                 "set of interface properties to forward");
}

Stage::~Stage() {
	delete pimpl_;
}

Stage::operator StagePrivate*() {
	return pimpl();
}

Stage::operator const StagePrivate*() const {
	return pimpl();
}

void Stage::reset() {
	auto impl = pimpl();
	// clear solutions + associated states
	impl->solutions_.clear();
	impl->failures_.clear();
	impl->num_failures_ = 0u;
	impl->states_.clear();
	// clear pull interfaces
	if (impl->starts_)
		impl->starts_->clear();
	if (impl->ends_)
		impl->ends_->clear();
	// reset push interfaces
	impl->prev_ends_.reset();
	impl->next_starts_.reset();
	// reset inherited properties
	impl->properties_.reset();
}

void Stage::init(const moveit::core::RobotModelConstPtr& robot_model) {
	// init properties once from parent
	auto impl = pimpl();
	impl->properties_.reset();
	if (impl->parent()) {
		try {
			ROS_DEBUG_STREAM_NAMED("Properties", "init '" << name() << "'");
			impl->properties_.performInitFrom(PARENT, impl->parent()->properties());
		} catch (const Property::error& e) {
			std::ostringstream oss;
			oss << e.what();
			// skip this stage and start error reporting at parent
			impl->parent()->pimpl()->composePropertyErrorMsg(e.name(), oss);
			throw InitStageException(*this, oss.str());
		}
	}
}

const ContainerBase* Stage::parent() const {
	return pimpl_->parent_;
}

const std::string& Stage::name() const {
	return pimpl_->name_;
}

void Stage::setName(const std::string& name) {
	pimpl_->name_ = name;
}

void Stage::forwardProperties(const InterfaceState& source, InterfaceState& dest) {
	const PropertyMap& src = source.properties();
	PropertyMap& dst = dest.properties();
	for (const auto& name : forwardedProperties()) {
		if (!src.hasProperty(name))
			continue;
		dst.set(name, src.get(name));
	}
}

Stage::SolutionCallbackList::const_iterator Stage::addSolutionCallback(SolutionCallback&& cb) {
	auto impl = pimpl();
	impl->solution_cbs_.emplace_back(std::move(cb));
	return --impl->solution_cbs_.cend();
}
void Stage::removeSolutionCallback(SolutionCallbackList::const_iterator which) {
	pimpl()->solution_cbs_.erase(which);
}

const ordered<SolutionBaseConstPtr>& Stage::solutions() const {
	return pimpl()->solutions_;
}

const std::list<SolutionBaseConstPtr>& Stage::failures() const {
	return pimpl()->failures_;
}

size_t Stage::numFailures() const {
	return pimpl()->num_failures_;
}

void Stage::silentFailure() {
	++(pimpl()->num_failures_);
}

bool Stage::storeFailures() const {
	return pimpl()->storeFailures();
}

PropertyMap& Stage::properties() {
	return pimpl()->properties_;
}

void Stage::setProperty(const std::string& name, const boost::any& value) {
	pimpl()->properties_.set(name, value);
}

double Stage::getTotalComputeTime() const {
	return pimpl()->total_compute_time_.count();
}

void StagePrivate::composePropertyErrorMsg(const std::string& property_name, std::ostream& os) {
	if (property_name.empty())
		return;
	os << "\nin stage '" << name() << "': ";
	try {
		const auto& p = properties_.property(property_name);
		if (p.defined()) {
			os << "defined here";
			return;
		} else
			os << "declared, but undefined";

		if (p.initsFrom(Stage::PARENT))
			os << ", inherits from parent";
		if (p.initsFrom(Stage::INTERFACE))
			os << ", initializes from interface";
	} catch (const Property::undeclared&) {
		os << "undeclared";
	}
	if (parent()->parent())
		parent()->pimpl()->composePropertyErrorMsg(property_name, os);
}

void Stage::reportPropertyError(const Property::error& e) {
	std::ostringstream oss;
	oss << e.what();
	pimpl()->composePropertyErrorMsg(e.name(), oss);
	throw std::runtime_error(oss.str());
}

std::ostream& operator<<(std::ostream& os, const StagePrivate& impl) {
	// starts
	for (const InterfaceConstPtr& i : { impl.prevEnds(), impl.starts() }) {
		os << std::setw(3);
		if (i)
			os << i->size();
		else
			os << "-";
	}
	// trajectories
	os << " " << flowSymbol<START_IF_MASK>(impl.interfaceFlags() | impl.requiredInterface()) << " ";
	os << std::setw(3) << impl.solutions_.size();
	os << " " << flowSymbol<END_IF_MASK>(impl.interfaceFlags() | impl.requiredInterface()) << " ";
	// ends
	for (const InterfaceConstPtr& i : { impl.ends(), impl.nextStarts() }) {
		os << std::setw(3);
		if (i)
			os << i->size();
		else
			os << "-";
	}
	// name
	os << " / " << impl.name();
	return os;
}

ComputeBase::ComputeBase(ComputeBasePrivate* impl) : Stage(impl) {}

PropagatingEitherWayPrivate::PropagatingEitherWayPrivate(PropagatingEitherWay* me, PropagatingEitherWay::Direction dir,
                                                         const std::string& name)
  : ComputeBasePrivate(me, name), configured_dir_(dir) {
	initInterface(dir);
}

// initialize pull interfaces to match requested propagation directions
void PropagatingEitherWayPrivate::initInterface(PropagatingEitherWay::Direction dir) {
	switch (dir) {
		case PropagatingEitherWay::FORWARD:
			required_interface_ = PROPAGATE_FORWARDS;
			if (!starts_)  // keep existing interface if possible
				starts_.reset(
				    new Interface([this](Interface::iterator it, bool /*updated*/) { this->dropFailedStarts(it); }));
			ends_.reset();
			return;
		case PropagatingEitherWay::BACKWARD:
			required_interface_ = PROPAGATE_BACKWARDS;
			starts_.reset();
			if (!ends_)  // keep existing interface if possible
				ends_.reset(new Interface([this](Interface::iterator it, bool /*updated*/) { this->dropFailedEnds(it); }));
			return;
		case PropagatingEitherWay::AUTO:
			required_interface_ = UNKNOWN;
			return;
	}
}

void PropagatingEitherWayPrivate::resolveInterface(InterfaceFlags expected) {
	if (expected == UNKNOWN)
		throw InitStageException(*me(), "cannot initialize to unknown interface");

	auto dir = PropagatingEitherWay::AUTO;
	if ((expected & START_IF_MASK) == READS_START || (expected & END_IF_MASK) == WRITES_NEXT_START)
		dir = PropagatingEitherWay::FORWARD;
	else if ((expected & START_IF_MASK) == WRITES_PREV_END || (expected & END_IF_MASK) == READS_END)
		dir = PropagatingEitherWay::BACKWARD;
	else {
		boost::format desc("propagator cannot satisfy expected interface %1% %2%");
		desc % flowSymbol<START_IF_MASK>(expected) % flowSymbol<END_IF_MASK>(expected);
		throw InitStageException(*me(), desc.str());
	}
	if (configured_dir_ != PropagatingEitherWay::AUTO && dir != configured_dir_) {
		boost::format desc("configured interface (%1% %2%) does not match expected one (%3% %4%)");
		desc % flowSymbol<START_IF_MASK>(required_interface_) % flowSymbol<END_IF_MASK>(required_interface_);
		desc % flowSymbol<START_IF_MASK>(expected) % flowSymbol<END_IF_MASK>(expected);
		throw InitStageException(*me(), desc.str());
	}
	initInterface(dir);
}

InterfaceFlags PropagatingEitherWayPrivate::requiredInterface() const {
	return required_interface_;
}

void PropagatingEitherWayPrivate::dropFailedStarts(Interface::iterator state) {
	if (std::isinf(state->priority().cost()))
		starts_->remove(state);
}
void PropagatingEitherWayPrivate::dropFailedEnds(Interface::iterator state) {
	if (std::isinf(state->priority().cost()))
		ends_->remove(state);
}

inline bool PropagatingEitherWayPrivate::hasStartState() const {
	return starts_ && !starts_->empty();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchStartState() {
	assert(hasStartState());
	return *starts_->remove(starts_->begin()).front();
}

inline bool PropagatingEitherWayPrivate::hasEndState() const {
	return ends_ && !ends_->empty();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchEndState() {
	assert(hasEndState());
	return *ends_->remove(ends_->begin()).front();
}

bool PropagatingEitherWayPrivate::canCompute() const {
	return hasStartState() || hasEndState();
}

void PropagatingEitherWayPrivate::compute() {
	PropagatingEitherWay* me = static_cast<PropagatingEitherWay*>(me_);

	if (hasStartState()) {
		const InterfaceState& state = fetchStartState();
		// enforce property initialization from INTERFACE
		properties_.performInitFrom(Stage::INTERFACE, state.properties());
		me->computeForward(state);
	}
	if (hasEndState()) {
		const InterfaceState& state = fetchEndState();
		// enforce property initialization from INTERFACE
		properties_.performInitFrom(Stage::INTERFACE, state.properties());
		me->computeBackward(state);
	}
}

PropagatingEitherWay::PropagatingEitherWay(const std::string& name)
  : PropagatingEitherWay(new PropagatingEitherWayPrivate(this, AUTO, name)) {}

PropagatingEitherWay::PropagatingEitherWay(PropagatingEitherWayPrivate* impl) : ComputeBase(impl) {}

void PropagatingEitherWay::restrictDirection(PropagatingEitherWay::Direction dir) {
	auto impl = pimpl();
	if (impl->configured_dir_ == dir)
		return;
	if (impl->configured_dir_ != AUTO)
		throw std::runtime_error("Cannot change direction after being connected");
	impl->configured_dir_ = dir;
	impl->initInterface(dir);
}

void PropagatingEitherWay::sendForward(const InterfaceState& from, InterfaceState&& to, SubTrajectory&& t) {
	pimpl()->sendForward(from, std::move(to), std::make_shared<SubTrajectory>(std::move(t)));
}

void PropagatingEitherWay::sendBackward(InterfaceState&& from, const InterfaceState& to, SubTrajectory&& t) {
	pimpl()->sendBackward(std::move(from), to, std::make_shared<SubTrajectory>(std::move(t)));
}

PropagatingForwardPrivate::PropagatingForwardPrivate(PropagatingForward* me, const std::string& name)
  : PropagatingEitherWayPrivate(me, PropagatingEitherWay::FORWARD, name) {
	// indicate, that we don't accept new states from ends_ interface
	ends_.reset();
}

PropagatingForward::PropagatingForward(const std::string& name)
  : PropagatingEitherWay(new PropagatingForwardPrivate(this, name)) {}

void PropagatingForward::computeBackward(const InterfaceState& /* to */) {
	assert(false);  // This should never be called
}

PropagatingBackwardPrivate::PropagatingBackwardPrivate(PropagatingBackward* me, const std::string& name)
  : PropagatingEitherWayPrivate(me, PropagatingEitherWay::BACKWARD, name) {
	// indicate, that we don't accept new states from starts_ interface
	starts_.reset();
}

PropagatingBackward::PropagatingBackward(const std::string& name)
  : PropagatingEitherWay(new PropagatingBackwardPrivate(this, name)) {}

void PropagatingBackward::computeForward(const InterfaceState& /* from */) {
	assert(false);  // This should never be called
}

GeneratorPrivate::GeneratorPrivate(Generator* me, const std::string& name) : ComputeBasePrivate(me, name) {}

InterfaceFlags GeneratorPrivate::requiredInterface() const {
	return InterfaceFlags(GENERATE);
}

bool GeneratorPrivate::canCompute() const {
	return static_cast<Generator*>(me_)->canCompute();
}

void GeneratorPrivate::compute() {
	static_cast<Generator*>(me_)->compute();
}

Generator::Generator(GeneratorPrivate* impl) : ComputeBase(impl) {}
Generator::Generator(const std::string& name) : Generator(new GeneratorPrivate(this, name)) {}

void Generator::spawn(InterfaceState&& state, SubTrajectory&& t) {
	pimpl()->spawn(std::move(state), std::make_shared<SubTrajectory>(std::move(t)));
}

MonitoringGeneratorPrivate::MonitoringGeneratorPrivate(MonitoringGenerator* me, const std::string& name)
  : GeneratorPrivate(me, name), monitored_(nullptr), registered_(false) {}

MonitoringGenerator::MonitoringGenerator(const std::string& name, Stage* monitored)
  : Generator(new MonitoringGeneratorPrivate(this, name)) {
	setMonitoredStage(monitored);
}

void MonitoringGenerator::setMonitoredStage(Stage* monitored) {
	auto impl = pimpl();
	if (impl->monitored_ == monitored)
		return;

	if (impl->monitored_ && impl->registered_) {
		impl->monitored_->removeSolutionCallback(impl->cb_);
		impl->registered_ = false;
	}

	impl->monitored_ = monitored;
}

void MonitoringGenerator::init(const moveit::core::RobotModelConstPtr& robot_model) {
	Generator::init(robot_model);

	auto impl = pimpl();
	if (!impl->monitored_)
		throw InitStageException(*this, "no monitored stage defined");
	if (!impl->registered_) {  // register only once
		impl->cb_ = impl->monitored_->addSolutionCallback(
		    std::bind(&MonitoringGeneratorPrivate::solutionCB, impl, std::placeholders::_1));
		impl->registered_ = true;
	}
}

void MonitoringGeneratorPrivate::solutionCB(const SolutionBase& s) {
	// forward only successful solutions to monitor
	if (!s.isFailure())
		static_cast<MonitoringGenerator*>(me())->onNewSolution(s);
}

ConnectingPrivate::ConnectingPrivate(Connecting* me, const std::string& name) : ComputeBasePrivate(me, name) {
	starts_.reset(new Interface(std::bind(&ConnectingPrivate::newState<Interface::BACKWARD>, this, std::placeholders::_1,
	                                      std::placeholders::_2)));
	ends_.reset(new Interface(std::bind(&ConnectingPrivate::newState<Interface::FORWARD>, this, std::placeholders::_1,
	                                    std::placeholders::_2)));
}

InterfaceFlags ConnectingPrivate::requiredInterface() const {
	return InterfaceFlags(CONNECT);
}

template <>
ConnectingPrivate::StatePair ConnectingPrivate::make_pair<Interface::BACKWARD>(Interface::const_iterator first,
                                                                               Interface::const_iterator second) {
	return std::make_pair(first, second);
}
template <>
ConnectingPrivate::StatePair ConnectingPrivate::make_pair<Interface::FORWARD>(Interface::const_iterator first,
                                                                              Interface::const_iterator second) {
	return std::make_pair(second, first);
}

template <Interface::Direction other>
void ConnectingPrivate::newState(Interface::iterator it, bool updated) {
	// TODO: only consider interface states with priority depth > threshold
	if (!std::isfinite(it->priority().cost())) {
		// remove pending pairs, if cost updated to infinity
		if (updated)
			pending.remove_if([it](const StatePair& p) { return p.first == it; });
		return;
	}
	if (updated) {
		// many pairs might be affected: sort
		pending.sort();
	} else {  // new state: insert all pairs with other interface
		InterfacePtr other_interface = pullInterface(other);
		for (Interface::iterator oit = other_interface->begin(), oend = other_interface->end(); oit != oend; ++oit) {
			if (!std::isfinite(oit->priority().cost()))
				break;
			if (static_cast<Connecting*>(me_)->compatible(*it, *oit))
				pending.insert(make_pair<other>(it, oit));
		}
	}
}

bool ConnectingPrivate::canCompute() const {
	return !pending.empty();
}

void ConnectingPrivate::compute() {
	const StatePair& top = pending.pop();
	const InterfaceState& from = *top.first;
	const InterfaceState& to = *top.second;
	static_cast<Connecting*>(me_)->compute(from, to);
}

Connecting::Connecting(const std::string& name) : ComputeBase(new ConnectingPrivate(this, name)) {}

void Connecting::reset() {
	pimpl()->pending.clear();
	ComputeBase::reset();
}

/// compare consistency of planning scenes
bool Connecting::compatible(const InterfaceState& from_state, const InterfaceState& to_state) const {
	const planning_scene::PlanningSceneConstPtr& from = from_state.scene();
	const planning_scene::PlanningSceneConstPtr& to = to_state.scene();

	if (from->getWorld()->size() != to->getWorld()->size()) {
		ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different number of collision objects");
		return false;
	}

	// both scenes should have the same set of collision objects, at the same location
	for (const auto& from_object_pair : *from->getWorld()) {
		const collision_detection::World::ObjectPtr& from_object = from_object_pair.second;
		const collision_detection::World::ObjectConstPtr& to_object = to->getWorld()->getObject(from_object_pair.first);
		if (!to_object) {
			ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": object missing: " << from_object_pair.first);
			return false;
		}
		if (from_object->shape_poses_.size() != to_object->shape_poses_.size()) {
			ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different object shapes: " << from_object_pair.first);
			return false;  // shapes not matching
		}

		for (auto from_it = from_object->shape_poses_.cbegin(), from_end = from_object->shape_poses_.cend(),
		          to_it = to_object->shape_poses_.cbegin();
		     from_it != from_end; ++from_it, ++to_it)
			if (!(from_it->matrix() - to_it->matrix()).isZero(1e-4)) {
				ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different object pose: " << from_object_pair.first);
				return false;  // transforms do not match
			}
	}

	// Also test for attached objects which have a different storage
	std::vector<const moveit::core::AttachedBody*> from_attached;
	std::vector<const moveit::core::AttachedBody*> to_attached;
	from->getCurrentState().getAttachedBodies(from_attached);
	to->getCurrentState().getAttachedBodies(to_attached);
	if (from_attached.size() != to_attached.size()) {
		ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different number of objects");
		return false;
	}

	for (const moveit::core::AttachedBody* from_object : from_attached) {
		auto it = std::find_if(to_attached.cbegin(), to_attached.cend(),
		                       [from_object](const moveit::core::AttachedBody* object) {
			                       return object->getName() == from_object->getName();
			                    });
		if (it == to_attached.cend()) {
			ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": object missing: " << from_object->getName());
			return false;
		}
		const moveit::core::AttachedBody* to_object = *it;
		if (from_object->getAttachedLink() != to_object->getAttachedLink()) {
			ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different attach links: " << from_object->getName()
			                                            << " attached to " << from_object->getAttachedLinkName() << " / "
			                                            << to_object->getAttachedLinkName());
			return false;  // links not matching
		}
		if (from_object->getFixedTransforms().size() != to_object->getFixedTransforms().size()) {
			ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different object shapes: " << from_object->getName());
			return false;  // shapes not matching
		}

		for (auto from_it = from_object->getFixedTransforms().cbegin(),
		          from_end = from_object->getFixedTransforms().cend(), to_it = to_object->getFixedTransforms().cbegin();
		     from_it != from_end; ++from_it, ++to_it)
			if (!(from_it->matrix() - to_it->matrix()).isZero(1e-4)) {
				ROS_DEBUG_STREAM_NAMED("Connecting", name() << ": different object pose: " << from_object->getName());
				return false;  // transforms do not match
			}
	}
	return true;
}

void Connecting::connect(const InterfaceState& from, const InterfaceState& to, const SolutionBasePtr& s) {
	pimpl()->connect(from, to, s);
}

std::ostream& operator<<(std::ostream& os, const Stage& stage) {
	os << *stage.pimpl();
	return os;
}
}  // namespace task_constructor
}  // namespace moveit
