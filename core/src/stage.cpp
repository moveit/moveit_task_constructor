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

#include <rclcpp/logging.hpp>

#include <fmt/core.h>
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
	static const char* msg = "Error initializing stage(s). RCLCPP_ERROR_STREAM(e) for details.";
	return msg;
}

std::ostream& operator<<(std::ostream& os, const InitStageException& e) {
	os << "Error initializing stage" << (e.errors_.size() > 1 ? "s" : "") << ":" << std::endl;
	for (const auto& pair : e.errors_)
		os << pair.first->name() << ": " << pair.second << std::endl;
	return os;
}

StagePrivate::StagePrivate(Stage* me, const std::string& name)
  : me_{ me }
  , name_{ name }
  , cost_term_{ std::make_unique<CostTerm>() }
  , total_compute_time_{}
  , parent_{ nullptr }
  , introspection_{ nullptr } {}

StagePrivate& StagePrivate::operator=(StagePrivate&& other) {
	assert(typeid(*this) == typeid(other));

	assert(states_.empty() && other.states_.empty());
	assert((!starts_ || starts_->empty()) && (!other.starts_ || other.starts_->empty()));
	assert((!ends_ || ends_->empty()) && (!other.ends_ || other.ends_->empty()));
	assert(solutions_.empty() && other.solutions_.empty());
	assert(failures_.empty() && other.failures_.empty());

	// me_ must not be changed!
	name_ = std::move(other.name_);
	properties_ = std::move(other.properties_);
	cost_term_ = std::move(other.cost_term_);
	solution_cbs_ = std::move(other.solution_cbs_);

	starts_ = std::move(other.starts_);
	ends_ = std::move(other.ends_);
	prev_ends_ = std::move(other.prev_ends_);
	next_starts_ = std::move(other.next_starts_);

	parent_ = other.parent_;
	it_ = other.it_;
	other.unparent();

	return *this;
}

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
	if ((required & actual) != required)
		throw InitStageException(*me(),
		                         fmt::format("actual interface {} {} does not match required interface {} {}",
		                                     flowSymbol<START_IF_MASK>(actual), flowSymbol<END_IF_MASK>(actual),
		                                     flowSymbol<START_IF_MASK>(required), flowSymbol<END_IF_MASK>(required)));
}

bool StagePrivate::storeSolution(const SolutionBasePtr& solution, const InterfaceState* from,
                                 const InterfaceState* to) {
	solution->setCreator(me());
	if (introspection_)
		introspection_->registerSolution(*solution);

	if (solution->isFailure()) {
		++num_failures_;
		if (parent())
			parent()->pimpl()->onNewFailure(*me(), from, to);
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

	computeCost(from, to, *solution);

	if (!storeSolution(solution, &from, nullptr))
		return;  // solution dropped

	me()->forwardProperties(from, to);

	auto to_it = states_.insert(states_.end(), std::move(to));

	// register stored interfaces with solution
	solution->setStartState(from);
	solution->setEndState(*to_it);

	if (!solution->isFailure())
		nextStarts()->add(*to_it);

	newSolution(solution);
}

void StagePrivate::sendBackward(InterfaceState&& from, const InterfaceState& to, const SolutionBasePtr& solution) {
	assert(prevEnds());

	computeCost(from, to, *solution);

	if (!storeSolution(solution, nullptr, &to))
		return;  // solution dropped

	me()->forwardProperties(to, from);

	auto from_it = states_.insert(states_.end(), std::move(from));

	solution->setStartState(*from_it);
	solution->setEndState(to);

	if (!solution->isFailure())
		prevEnds()->add(*from_it);

	newSolution(solution);
}

void StagePrivate::spawn(InterfaceState&& from, InterfaceState&& to, const SolutionBasePtr& solution) {
	assert(prevEnds() && nextStarts());

	computeCost(from, to, *solution);

	if (!storeSolution(solution, nullptr, nullptr))
		return;  // solution dropped

	auto from_it = states_.insert(states_.end(), std::move(from));
	auto to_it = states_.insert(states_.end(), std::move(to));

	solution->setStartState(*from_it);
	solution->setEndState(*to_it);

	if (!solution->isFailure()) {
		prevEnds()->add(*from_it);
		nextStarts()->add(*to_it);
	}

	newSolution(solution);
}

void StagePrivate::spawn(InterfaceState&& state, const SolutionBasePtr& solution) {
	spawn(InterfaceState(state), std::move(state), solution);
}

void StagePrivate::connect(const InterfaceState& from, const InterfaceState& to, const SolutionBasePtr& solution) {
	computeCost(from, to, *solution);

	if (!storeSolution(solution, &from, &to))
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

// To solve the chicken-egg problem in computeCost() and provide proper states at both ends of the solution,
// this class temporarily sets the new interface states w/o registering the solution yet.
// On destruction the start/end states are reset again.
struct TmpSolutionContext
{
	SolutionBase& solution_;
	TmpSolutionContext(SolutionBase& solution, Stage* creator, const InterfaceState& from, const InterfaceState& to)
	  : solution_(solution) {
		assert(solution_.start_ == nullptr);
		assert(solution_.end_ == nullptr);
		solution_.start_ = &from;
		solution_.end_ = &to;
		solution_.creator_ = creator;
	}
	~TmpSolutionContext() {
		solution_.start_ = nullptr;
		solution_.end_ = nullptr;
		solution_.creator_ = nullptr;
	}
};
void StagePrivate::computeCost(const InterfaceState& from, const InterfaceState& to, SolutionBase& solution) {
	// no reason to compute costs for a failed solution
	if (solution.isFailure())
		return;

	// Temporarily set start/end states of the solution w/o actually registering the solution with them
	// This allows CostTerms to compute costs based on the InterfaceState.
	TmpSolutionContext tip(solution, me(), from, to);

	std::string comment;
	assert(cost_term_);
	solution.setCost(solution.computeCost(*cost_term_, comment));

	// If a comment was specified, add it to the solution
	if (!comment.empty() && !solution.comment().empty()) {
		solution.setComment(solution.comment() + " (" + comment + ")");
	} else if (!comment.empty()) {
		solution.setComment(comment);
	}
}

Stage::Stage(StagePrivate* impl) : pimpl_(impl) {
	assert(impl);
	auto& p = properties();
	p.declare<double>("timeout", "timeout per run (s)");
	p.declare<std::string>("marker_ns", name(), "marker namespace");
	p.declare<TrajectoryExecutionInfo>("trajectory_execution_info", TrajectoryExecutionInfo(),
	                                   "settings used when executing the trajectory");

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
	impl->total_compute_time_ = std::chrono::duration<double>::zero();
}

void Stage::init(const moveit::core::RobotModelConstPtr& /* robot_model */) {
	auto impl = pimpl();

	// init properties once from parent
	impl->properties_.reset();
	if (impl->parent()) {
		try {
			RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Properties"), fmt::format("init '{}'", name()));
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

uint32_t Stage::introspectionId() const {
	if (!pimpl_->introspection_)
		throw std::runtime_error("Task is not initialized yet or Introspection was disabled.");
	return const_cast<const moveit::task_constructor::Introspection*>(pimpl_->introspection_)->stageId(this);
}
Introspection* Stage::introspection() const {
	return pimpl_->introspection_;
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

void Stage::setCostTerm(const CostTermConstPtr& term) {
	if (!term)
		pimpl()->cost_term_ = std::make_unique<CostTerm>();
	else
		pimpl()->cost_term_ = term;
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

void Stage::explainFailure(std::ostream& os) const {
	if (!failures().empty())
		os << ": " << failures().front()->comment();
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
				starts_ = std::make_shared<Interface>();
			ends_.reset();
			return;
		case PropagatingEitherWay::BACKWARD:
			required_interface_ = PROPAGATE_BACKWARDS;
			starts_.reset();
			if (!ends_)  // keep existing interface if possible
				ends_ = std::make_shared<Interface>();
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
	else
		throw InitStageException(*me(),
		                         fmt::format("propagator cannot satisfy expected interface {} {}",
		                                     flowSymbol<START_IF_MASK>(expected), flowSymbol<END_IF_MASK>(expected)));
	if (configured_dir_ != PropagatingEitherWay::AUTO && dir != configured_dir_)
		throw InitStageException(*me(),
		                         fmt::format("configured interface ({} {}) does not match expected one ({} {})",
		                                     flowSymbol<START_IF_MASK>(required_interface_),
		                                     flowSymbol<END_IF_MASK>(required_interface_),
		                                     flowSymbol<START_IF_MASK>(expected), flowSymbol<END_IF_MASK>(expected)));
	initInterface(dir);
}

InterfaceFlags PropagatingEitherWayPrivate::requiredInterface() const {
	return required_interface_;
}

inline bool PropagatingEitherWayPrivate::hasStartState() const {
	return starts_ && !starts_->empty() && starts_->front()->priority().enabled();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchStartState() {
	assert(hasStartState());
	return *starts_->remove(starts_->begin()).front();
}

inline bool PropagatingEitherWayPrivate::hasEndState() const {
	return ends_ && !ends_->empty() && ends_->front()->priority().enabled();
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

template <Interface::Direction dir>
void PropagatingEitherWay::send(const InterfaceState& start, InterfaceState&& end, SubTrajectory&& trajectory) {
	pimpl()->send<dir>(start, std::move(end), std::make_shared<SubTrajectory>(std::move(trajectory)));
}
// Explicit template instantiation is required. The compiler, otherwise, might just inline them.
template void PropagatingEitherWay::send<Interface::FORWARD>(const InterfaceState& start, InterfaceState&& end,
                                                             SubTrajectory&& trajectory);
template void PropagatingEitherWay::send<Interface::BACKWARD>(const InterfaceState& start, InterfaceState&& end,
                                                              SubTrajectory&& trajectory);

void PropagatingEitherWay::computeForward(const InterfaceState& from) {
	computeGeneric<Interface::FORWARD>(from);
}

void PropagatingEitherWay::computeBackward(const InterfaceState& to) {
	computeGeneric<Interface::BACKWARD>(to);
}

template <Interface::Direction dir>
void PropagatingEitherWay::computeGeneric(const InterfaceState& start) {
	planning_scene::PlanningScenePtr end;
	SubTrajectory trajectory;

	if (!compute(start, end, trajectory, dir) && trajectory.comment().empty())
		silentFailure();  // there is nothing to report (comment is empty)
	else
		send<dir>(start, InterfaceState(end), std::move(trajectory));
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

void Generator::spawn(InterfaceState&& from, InterfaceState&& to, SubTrajectory&& t) {
	pimpl()->spawn(std::move(from), std::move(to), std::make_shared<SubTrajectory>(std::move(t)));
}

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
	starts_ = std::make_shared<Interface>(std::bind(&ConnectingPrivate::newState<Interface::BACKWARD>, this,
	                                                std::placeholders::_1, std::placeholders::_2));
	ends_ = std::make_shared<Interface>(
	    std::bind(&ConnectingPrivate::newState<Interface::FORWARD>, this, std::placeholders::_1, std::placeholders::_2));
}

InterfaceFlags ConnectingPrivate::requiredInterface() const {
	return InterfaceFlags(CONNECT);
}

template <>
ConnectingPrivate::StatePair ConnectingPrivate::make_pair<Interface::BACKWARD>(Interface::const_iterator first,
                                                                               Interface::const_iterator second) {
	return StatePair(first, second);
}
template <>
ConnectingPrivate::StatePair ConnectingPrivate::make_pair<Interface::FORWARD>(Interface::const_iterator first,
                                                                              Interface::const_iterator second) {
	return StatePair(second, first);
}

template <Interface::Direction dir>
void ConnectingPrivate::newState(Interface::iterator it, Interface::UpdateFlags updated) {
	auto parent_pimpl = parent()->pimpl();
	// disable current interface to break loop (jumping back and forth between both interfaces)
	// this will be checked by notifyEnabled() below
	Interface::DisableNotify disable_source_interface(*pullInterface<dir>());
	if (updated) {
		if (updated.testFlag(Interface::STATUS) &&  // only perform these costly operations if needed
		    pullInterface<opposite<dir>()>()->notifyEnabled())  // suppressing recursive loop?
		{
			// If status has changed, propagate the update to the opposite side
			auto status = it->priority().status();
			if (status == InterfaceState::Status::PRUNED)  // PRUNED becomes ARMED on opposite side
				status = InterfaceState::Status::ARMED;  // (only for pending state pairs)

			for (const auto& candidate : this->pending) {
				if (std::get<opposite<dir>()>(candidate) != it)  // only consider pairs with source state == state
					continue;
				auto oit = std::get<dir>(candidate);  // opposite target state
				auto ostatus = oit->priority().status();
				if (ostatus != status) {
					if (status != InterfaceState::Status::ENABLED) {
						// quicker check for hasPendingOpposites(): search in it->owner() for an enabled alternative
						bool cancel = false;  // if found, cancel propagation of new status
						for (const auto alternative : *it->owner())
							if ((cancel = alternative->priority().enabled()))
								break;
						if (cancel)
							continue;
					}
					// pass creator=nullptr to skip hasPendingOpposites() check
					parent_pimpl->setStatus<opposite<dir>()>(nullptr, nullptr, &*oit, status);
				}
			}
		}

		// many pairs will have changed priorities: resort pending list
		pending.sort();
	} else {  // new state: insert all pairs with other interface
		assert(it->priority().enabled());  // new solutions are feasible, aren't they?
		InterfacePtr other_interface = pullInterface<dir>();
		bool have_enabled_opposites = false;

		// other interface states to re-enable (post-poned because otherwise order in other_interface changes during loop)
		std::vector<Interface::iterator> oit_to_enable;
		for (Interface::iterator oit = other_interface->begin(), oend = other_interface->end(); oit != oend; ++oit) {
			if (!static_cast<Connecting*>(me_)->compatible(*it, *oit))
				continue;

			// re-enable the opposing state oit (and its associated solution branch) if its status is ARMED
			// https://github.com/moveit/moveit_task_constructor/pull/309#issuecomment-974636202
			if (oit->priority().status() == InterfaceState::Status::ARMED) {
				oit_to_enable.push_back(oit);
				have_enabled_opposites = true;
			}
			if (oit->priority().enabled())
				have_enabled_opposites = true;

			// Remember all pending states, regardless of their status!
			pending.insert(make_pair<dir>(it, oit));
		}
		// actually re-enable other interface states, which were scheduled for re-enabling above
		for (Interface::iterator oit : oit_to_enable)
			parent_pimpl->setStatus<opposite<dir>()>(me(), &*it, &*oit, InterfaceState::Status::ENABLED);

		if (!have_enabled_opposites)  // prune new state and associated branch if necessary
			// pass creator=nullptr to skip hasPendingOpposites() check as we did this here already
			parent_pimpl->setStatus<dir>(nullptr, nullptr, &*it, InterfaceState::Status::ARMED);
	}
#if 0
	auto& os = std::cerr;
	for (auto d : { Interface::FORWARD, Interface::BACKWARD }) {
		if (d == Interface::FORWARD)
			os << "  " << std::setw(10) << std::left << this->name();
		else
			os << std::setw(12) << std::right << "";
		if (dir != d)
			os << (updated ? " !" : " +");
		else
			os << "  ";
		os << d << " " << this->pullInterface(d) << ": " << *this->pullInterface(d) << std::endl;
	}
	os << std::setw(15) << " ";
	os << pendingPairsPrinter() << std::endl;
#endif
}

// Check whether there are pending feasible states (other than source) that could connect to target.
// If not, we exhausted all solution candidates for target and thus should mark it as failure.
template <Interface::Direction dir>
inline bool ConnectingPrivate::hasPendingOpposites(const InterfaceState* source, const InterfaceState* target) const {
	for (const auto& candidate : this->pending) {
		static_assert(Interface::FORWARD == 0 && Interface::BACKWARD == 1,
		              "This code assumes FORWARD=0, BACKWARD=1. Don't change their order!");
		const InterfaceState* src = &*std::get<dir>(candidate);
		const InterfaceState* tgt = &*std::get<opposite<dir>()>(candidate);

		if (tgt == target && src != source && src->priority().enabled())
			return true;

		// early stopping when only infeasible pairs are to come
		if (!std::get<0>(candidate)->priority().enabled() || !std::get<1>(candidate)->priority().enabled())
			break;
	}
	return false;
}
// explicitly instantiate templates for both directions
template bool ConnectingPrivate::hasPendingOpposites<Interface::FORWARD>(const InterfaceState* start,
                                                                         const InterfaceState* end) const;
template bool ConnectingPrivate::hasPendingOpposites<Interface::BACKWARD>(const InterfaceState* end,
                                                                          const InterfaceState* start) const;

bool ConnectingPrivate::canCompute() const {
	// ROS_DEBUG_STREAM("canCompute " << name() << ": " << pendingPairsPrinter());
	// Do we still have feasible pending state pairs?
	return !pending.empty() && pending.front().first->priority().enabled() &&
	       pending.front().second->priority().enabled();
}

void ConnectingPrivate::compute() {
	const StatePair& top = pending.pop();
	const InterfaceState& from = *top.first;
	const InterfaceState& to = *top.second;
	assert(from.priority().enabled() && to.priority().enabled());
	static_cast<Connecting*>(me_)->compute(from, to);
}

std::ostream& operator<<(std::ostream& os, const PendingPairsPrinter& p) {
	const auto* impl = p.instance_;
	const char* reset = InterfaceState::colorForStatus(3);
	for (const auto& candidate : impl->pending) {
		size_t first = getIndex(*impl->starts(), candidate.first);
		size_t second = getIndex(*impl->ends(), candidate.second);
		os << InterfaceState::colorForStatus(candidate.first->priority().status()) << first << reset << ":"
		   << InterfaceState::colorForStatus(candidate.second->priority().status()) << second << reset << " ";
	}
	if (impl->pending.empty())
		os << "---";
	return os;
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

	auto false_with_debug = [](auto... args) {
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("Connecting"), fmt::format(args...));
		return false;
	};

	if (from->getWorld()->size() != to->getWorld()->size())
		return false_with_debug("{}: different number of collision objects", name());

	// both scenes should have the same set of collision objects, at the same location
	for (const auto& from_object_pair : *from->getWorld()) {
		const std::string& from_object_name = from_object_pair.first;
		const collision_detection::World::ObjectPtr& from_object = from_object_pair.second;
		const collision_detection::World::ObjectConstPtr& to_object = to->getWorld()->getObject(from_object_name);
		if (!to_object)
			return false_with_debug("{}: object missing: {}", name(), from_object_name);

		if (!(from_object->pose_.matrix() - to_object->pose_.matrix()).isZero(1e-4))
			return false_with_debug("{}: different object pose: {}", name(), from_object_name);

		if (from_object->shape_poses_.size() != to_object->shape_poses_.size())
			return false_with_debug("{}: different object shapes: {}", name(), from_object_name);

		for (auto from_it = from_object->shape_poses_.cbegin(), from_end = from_object->shape_poses_.cend(),
		          to_it = to_object->shape_poses_.cbegin();
		     from_it != from_end; ++from_it, ++to_it)
			if (!(from_it->matrix() - to_it->matrix()).isZero(1e-4))
				return false_with_debug("{}: different shape pose: {}", name(), from_object_name);
	}

	// Also test for attached objects which have a different storage
	std::vector<const moveit::core::AttachedBody*> from_attached;
	std::vector<const moveit::core::AttachedBody*> to_attached;
	from->getCurrentState().getAttachedBodies(from_attached);
	to->getCurrentState().getAttachedBodies(to_attached);
	if (from_attached.size() != to_attached.size())
		return false_with_debug("{}: different number of objects", name());

	for (const moveit::core::AttachedBody* from_object : from_attached) {
		auto it = std::find_if(to_attached.cbegin(), to_attached.cend(),
		                       [from_object](const moveit::core::AttachedBody* object) {
			                       return object->getName() == from_object->getName();
		                       });
		if (it == to_attached.cend())
			return false_with_debug("{}: object missing: {}", name(), from_object->getName());

		const moveit::core::AttachedBody* to_object = *it;
		if (from_object->getAttachedLink() != to_object->getAttachedLink())
			return false_with_debug("{}: different attach links: {} attached to {} vs. {}",  //
			                        name(), from_object->getName(),  //
			                        from_object->getAttachedLink()->getName(), to_object->getAttachedLink()->getName());

		if (from_object->getShapes().size() != to_object->getShapes().size())
			return false_with_debug("{}: different object shapes: {}", name(), from_object->getName());

		auto from_it = from_object->getShapePosesInLinkFrame().cbegin();
		auto from_end = from_object->getShapePosesInLinkFrame().cend();
		auto to_it = to_object->getShapePosesInLinkFrame().cbegin();
		for (; from_it != from_end; ++from_it, ++to_it)
			if (!(from_it->matrix() - to_it->matrix()).isZero(1e-4))
				return false_with_debug("{}: different pose of attached object shape: {}", name(), from_object->getName());
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
