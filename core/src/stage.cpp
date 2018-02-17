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
#include <iostream>
#include <iomanip>
#include <ros/console.h>

using namespace std::placeholders;
namespace moveit { namespace task_constructor {

void InitStageException::push_back(const Stage &stage, const std::string &msg)
{
	errors_.emplace_back(std::make_pair(&stage, msg));
}

void InitStageException::append(InitStageException &other)
{
	errors_.splice(errors_.end(), other.errors_);
}

const char *InitStageException::what() const noexcept
{
	static const char* msg = "Error initializing stage(s)";
	return msg;
}

std::ostream& operator<<(std::ostream &os, const InitStageException& e) {
	os << e.what() << std::endl;
	for (const auto &pair : e.errors_)
		os << pair.first->name() << ": " << pair.second << std::endl;
	return os;
}


StagePrivate::StagePrivate(Stage *me, const std::string &name)
   : me_(me), name_(name), parent_(nullptr), introspection_(nullptr)
{}

InterfaceFlags StagePrivate::interfaceFlags() const
{
	InterfaceFlags f;
	if (starts())  f |= READS_START;
	if (ends()) f |= READS_END;
	if (prevEnds()) f |= WRITES_PREV_END;
	if (nextStarts()) f |= WRITES_NEXT_START;
	return f;
}

void StagePrivate::newSolution(const SolutionBase &solution)
{
	if (introspection_)
		introspection_->registerSolution(solution);

	// call solution callbacks for both, valid solutions and failures
	for (const auto& cb : solution_cbs_)
		cb(solution);

	if (parent())
		parent()->onNewSolution(solution);
}

Stage::Stage(StagePrivate *impl)
   : pimpl_(impl)
{
	assert(impl);
}

Stage::~Stage()
{
	delete pimpl_;
}

Stage::operator StagePrivate *() {
	return pimpl();
}

Stage::operator const StagePrivate*() const {
	return pimpl();
}

void Stage::reset()
{
	auto impl = pimpl();
	if (impl->starts_) impl->starts_->clear();
	if (impl->ends_) impl->ends_->clear();
	impl->prev_ends_.reset();
	impl->next_starts_.reset();
}

void Stage::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	// init properties once from parent
	auto impl = pimpl();
	impl->properties_.reset();
	if (impl->parent())
		impl->properties_.performInitFrom(PARENT, impl->parent()->properties());
}

const ContainerBase *Stage::parent() const {
	return pimpl_->parent_;
}

const std::string& Stage::name() const {
	return pimpl_->name_;
}

void Stage::setName(const std::string& name)
{
	pimpl_->name_ = name;
}

bool Stage::storeFailures() const
{
	return pimpl_->introspection_ != nullptr;
}

Stage::SolutionCallbackList::const_iterator Stage::addSolutionCallback(SolutionCallback &&cb)
{
	auto impl = pimpl();
	impl->solution_cbs_.emplace_back(std::move(cb));
	return --impl->solution_cbs_.cend();
}
void Stage::removeSolutionCallback(SolutionCallbackList::const_iterator which)
{
	pimpl()->solution_cbs_.erase(which);
}

PropertyMap &Stage::properties()
{
	return pimpl()->properties_;
}

void Stage::setProperty(const std::string& name, const boost::any& value) {
	pimpl()->properties_.set(name, value);
}

template<InterfaceFlag own, InterfaceFlag other>
const char* direction(const StagePrivate& stage) {
	InterfaceFlags f = stage.interfaceFlags();
	bool own_if = f & own;
	bool other_if = f & other;
	bool reverse = own & INPUT_IF_MASK;
	if (own_if && other_if) return "<>";
	if (!own_if && !other_if) return "--";
	if (other_if ^ reverse) return "->";
	return "<-";
}

std::ostream& operator<<(std::ostream &os, const Stage& stage) {
	auto impl = stage.pimpl();
	// starts
	for (const InterfaceConstPtr& i : {impl->prevEnds(), impl->starts()}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// trajectories
	os << std::setw(5) << direction<READS_START, WRITES_PREV_END>(*impl)
	   << std::setw(3) << stage.numSolutions()
	   << std::setw(5) << direction<READS_END, WRITES_NEXT_START>(*impl);
	// ends
	for (const InterfaceConstPtr& i : {impl->ends(), impl->nextStarts()}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// name
	os << " / " << stage.name();
	return os;
}

SubTrajectory& ComputeBasePrivate::addTrajectory(SubTrajectory&& trajectory) {
	trajectory.setCreator(this);
	if (!trajectory.isFailure()) {
		return *solutions_.insert(std::move(trajectory));
	} else if (me()->storeFailures()) {
		// only store failures when introspection is enabled
		auto it = failures_.insert(failures_.end(), std::move(trajectory));
		return *it;
	} else
		return trajectory;
}


ComputeBase::ComputeBase(ComputeBasePrivate *impl)
   : Stage(impl)
{
}

size_t ComputeBase::numSolutions() const {
	return pimpl()->solutions_.size();
}

size_t ComputeBase::numFailures() const
{
	return pimpl()->num_failures_;
}

void ComputeBase::processSolutions(const Stage::SolutionProcessor &processor) const
{
	for (const auto& s : pimpl()->solutions_)
		if (!processor(s))
			return;
}

void ComputeBase::processFailures(const Stage::SolutionProcessor &processor) const
{
	for (const auto& s : pimpl()->failures_)
		if (!processor(s))
			return;
}

void ComputeBase::reset() {
	pimpl()->solutions_.clear();
	pimpl()->failures_.clear();
	Stage::reset();
}


PropagatingEitherWayPrivate::PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir, const std::string &name)
   : ComputeBasePrivate(me, name), dir(dir)
{
}

void PropagatingEitherWayPrivate::dropFailedStarts(Interface::iterator state) {
	// move infinite-cost states to processed list
	if (std::isinf(state->priority().cost()))
		starts_->moveTo(state, processed, processed.end());
}
void PropagatingEitherWayPrivate::dropFailedEnds(Interface::iterator state) {
	// move infinite-cost states to processed list
	if (std::isinf(state->priority().cost()))
		ends_->moveTo(state, processed, processed.end());
}

inline bool PropagatingEitherWayPrivate::hasStartState() const{
	return starts_ && !starts_->empty();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchStartState(){
	assert(hasStartState());
	// move state to processed list
	return *starts_->moveTo(starts_->begin(), processed, processed.end());
}

inline bool PropagatingEitherWayPrivate::hasEndState() const{
	return ends_ && !ends_->empty();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchEndState(){
	assert(hasEndState());
	// move state to processed list
	return *ends_->moveTo(ends_->begin(), processed, processed.end());
}

bool PropagatingEitherWayPrivate::canCompute() const
{
	if ((dir & PropagatingEitherWay::FORWARD) && hasStartState())
		return true;
	if ((dir & PropagatingEitherWay::BACKWARD) && hasEndState())
		return true;
	return false;
}

bool PropagatingEitherWayPrivate::compute()
{
	PropagatingEitherWay* me = static_cast<PropagatingEitherWay*>(me_);

	bool result = false;
	if ((dir & PropagatingEitherWay::FORWARD) && hasStartState()) {
		const InterfaceState& state = fetchStartState();
		// enforce property initialization from INTERFACE
		properties_.performInitFrom(Stage::INTERFACE, state.properties(), true);
		if (countFailures(me->computeForward(state)))
			result |= true;
	}
	if ((dir & PropagatingEitherWay::BACKWARD) && hasEndState()) {
		const InterfaceState& state = fetchEndState();
		// enforce property initialization from INTERFACE
		properties_.performInitFrom(Stage::INTERFACE, state.properties(), true);
		if (countFailures(me->computeBackward(state)))
			result |= true;
	}
	return result;
}


PropagatingEitherWay::PropagatingEitherWay(const std::string &name)
   : PropagatingEitherWay(new PropagatingEitherWayPrivate(this, ANYWAY, name))
{
}

PropagatingEitherWay::PropagatingEitherWay(PropagatingEitherWayPrivate *impl)
   : ComputeBase(impl)
{
	initInterface();
}

void PropagatingEitherWay::initInterface()
{
	auto impl = pimpl();
	if (impl->dir & PropagatingEitherWay::FORWARD) {
		if (!impl->starts_)  // keep existing interface if possible
			impl->starts_.reset(new Interface(std::bind(&PropagatingEitherWayPrivate::dropFailedStarts, impl, _1)));
	} else {
		impl->starts_.reset();
	}

	if (impl->dir & PropagatingEitherWay::BACKWARD) {
		if (!impl->ends_)  // keep existing interface if possible
			impl->ends_.reset(new Interface(std::bind(&PropagatingEitherWayPrivate::dropFailedEnds, impl, _1)));
	} else {
		impl->ends_.reset();
	}
}

void PropagatingEitherWay::restrictDirection(PropagatingEitherWay::Direction dir)
{
	auto impl = pimpl();
	if (impl->dir == dir) return;
	if (impl->isConnected())
		throw std::runtime_error("Cannot change direction after being connected");
	impl->dir = dir;
	initInterface();
}

void PropagatingEitherWay::reset()
{
	pimpl()->processed.clear();
	ComputeBase::reset();
}

void PropagatingEitherWay::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	ComputeBase::init(scene);

	auto impl = pimpl();

	// after being connected, restrict actual interface directions
	if (!impl->nextStarts())
		impl->starts_.reset();
	if (!impl->prevEnds())
		impl->ends_.reset();
	if (!impl->isConnected())
		throw InitStageException(*this, "can neither send forwards nor backwards");
}

void PropagatingEitherWay::sendForward(const InterfaceState& from,
                                       InterfaceState&& to,
                                       SubTrajectory&& t) {
	auto impl = pimpl();
	SubTrajectory &trajectory = impl->addTrajectory(std::move(t));
	trajectory.setStartState(from);
	impl->nextStarts()->add(std::move(to), &trajectory, NULL);
	impl->newSolution(trajectory);
}

void PropagatingEitherWay::sendBackward(InterfaceState&& from,
                                        const InterfaceState& to,
                                        SubTrajectory&& t) {
	auto impl = pimpl();
	SubTrajectory& trajectory = impl->addTrajectory(std::move(t));
	trajectory.setEndState(to);
	impl->prevEnds()->add(std::move(from), NULL, &trajectory);
	impl->newSolution(trajectory);
}


PropagatingForwardPrivate::PropagatingForwardPrivate(PropagatingForward *me, const std::string &name)
   : PropagatingEitherWayPrivate(me, PropagatingEitherWay::FORWARD, name)
{
	// indicate, that we don't accept new states from ends_ interface
	ends_.reset();
}


PropagatingForward::PropagatingForward(const std::string& name)
   : PropagatingEitherWay(new PropagatingForwardPrivate(this, name))
{}

bool PropagatingForward::computeBackward(const InterfaceState &to)
{
	assert(false); // This should never be called
}


PropagatingBackwardPrivate::PropagatingBackwardPrivate(PropagatingBackward *me, const std::string &name)
   : PropagatingEitherWayPrivate(me, PropagatingEitherWay::BACKWARD, name)
{
	// indicate, that we don't accept new states from starts_ interface
	starts_.reset();
}


PropagatingBackward::PropagatingBackward(const std::string &name)
   : PropagatingEitherWay(new PropagatingBackwardPrivate(this, name))
{}

bool PropagatingBackward::computeForward(const InterfaceState &from)
{
	assert(false); // This should never be called
}


GeneratorPrivate::GeneratorPrivate(Generator *me, const std::string &name)
   : ComputeBasePrivate(me, name)
{}

bool GeneratorPrivate::canCompute() const {
	return static_cast<Generator*>(me_)->canCompute();
}

bool GeneratorPrivate::compute() {
	return countFailures(static_cast<Generator*>(me_)->compute());
}


Generator::Generator(const std::string &name)
   : ComputeBase(new GeneratorPrivate(this, name))
{}

void Generator::spawn(InterfaceState&& state, SubTrajectory&& t)
{
	assert(state.incomingTrajectories().empty() &&
	       state.outgoingTrajectories().empty());
	assert(!t.trajectory());

	auto impl = pimpl();
	SubTrajectory& trajectory = impl->addTrajectory(std::move(t));
	impl->prevEnds()->add(InterfaceState(state), NULL, &trajectory);
	impl->nextStarts()->add(std::move(state), &trajectory, NULL);
	impl->newSolution(trajectory);
}


ConnectingPrivate::ConnectingPrivate(Connecting *me, const std::string &name)
   : ComputeBasePrivate(me, name)
{
	starts_.reset(new Interface(std::bind(&ConnectingPrivate::newStartState, this, _1, _2)));
	ends_.reset(new Interface(std::bind(&ConnectingPrivate::newEndState, this, _1, _2)));
}

void ConnectingPrivate::newStartState(Interface::iterator it, bool updated)
{
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
	} else { // new state: insert all pairs with other interface
		for (auto oit = ends_->begin(), oend = ends_->end(); oit != oend; ++oit) {
			if (!std::isfinite(oit->priority().cost()))
				break;
			pending.insert(std::make_pair(it, oit));
		}
	}
}

void ConnectingPrivate::newEndState(Interface::iterator it, bool updated)
{
	if (!std::isfinite(it->priority().cost())) {
		// remove pending pairs, if cost updated to infinity
		if (updated)
			pending.remove_if([it](const StatePair& p) { return p.second == it; });
		return;
	}
	if (updated) {
		// many pairs might be affected: sort
		pending.sort();
	} else { // new state: insert all pairs with other interface
		for (auto oit = starts_->begin(), oend = starts_->end(); oit != oend; ++oit) {
			if (!std::isfinite(oit->priority().cost()))
				break;
			pending.insert(std::make_pair(oit, it));
		}
	}
}

bool ConnectingPrivate::canCompute() const{
	return !pending.empty();
}

bool ConnectingPrivate::compute() {
	const StatePair& top = pending.pop();
	const InterfaceState& from = *top.first;
	const InterfaceState& to = *top.second;
	return countFailures(static_cast<Connecting*>(me_)->compute(from, to));
}


Connecting::Connecting(const std::string &name)
   : ComputeBase(new ConnectingPrivate(this, name))
{
}

void Connecting::reset()
{
	pimpl()->pending.clear();
	ComputeBase::reset();
}

void Connecting::connect(const InterfaceState& from, const InterfaceState& to,
                         SubTrajectory&& t) {
	auto impl = pimpl();
	SubTrajectory& trajectory = impl->addTrajectory(std::move(t));
	trajectory.setStartState(from);
	trajectory.setEndState(to);
	impl->newSolution(trajectory);
}

} }
