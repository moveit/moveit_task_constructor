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
#include <iostream>
#include <iomanip>
#include <ros/console.h>

namespace moveit { namespace task_constructor {

SubTrajectory::SubTrajectory(StagePrivate *creator, const robot_trajectory::RobotTrajectoryPtr &traj, double cost)
   : SolutionBase(creator, cost), trajectory_(traj)
{}


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
   : me_(me), name_(name), parent_(nullptr)
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

Stage::SolutionCallbackList::const_iterator Stage::addSolutionCallback(SolutionCallback &&cb)
{
	auto impl = pimpl();
	impl->solution_cbs_.emplace_back(std::move(cb));
	return --impl->solution_cbs_.cend();
}

void Stage::erase(SolutionCallbackList::const_iterator which)
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


ComputeBase::ComputeBase(ComputeBasePrivate *impl)
   : Stage(impl)
{
}

SubTrajectory& ComputeBase::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	auto &trajs = pimpl()->trajectories_;
	trajs.emplace_back(SubTrajectory(pimpl(), trajectory, cost));
	return trajs.back();
}

size_t ComputeBase::numSolutions() const {
	return pimpl()->trajectories_.size();
}

void ComputeBase::processSolutions(const Stage::SolutionProcessor &processor) const
{
	for (const auto& s : pimpl()->trajectories_)
		if (!processor(s))
			return;
}

void ComputeBase::processFailures(const Stage::SolutionProcessor &processor) const
{
	// TODO
}

void ComputeBase::reset() {
	pimpl()->trajectories_.clear();
	Stage::reset();
}


PropagatingEitherWayPrivate::PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir, const std::string &name)
   : ComputeBasePrivate(me, name), dir(dir)
{
}

inline bool PropagatingEitherWayPrivate::hasStartState() const{
	return next_start_state_ != starts_->end();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchStartState(){
	if (!hasStartState())
		throw std::runtime_error("no new state for beginning available");

	const InterfaceState& state= *next_start_state_;
	++next_start_state_;

	return state;
}

inline bool PropagatingEitherWayPrivate::hasEndState() const{
	return next_end_state_ != ends_->end();
}

const InterfaceState& PropagatingEitherWayPrivate::fetchEndState(){
	if(!hasEndState())
		throw std::runtime_error("no new state for ending available");

	const InterfaceState& state= *next_end_state_;
	++next_end_state_;

	return state;
}

void PropagatingEitherWayPrivate::initProperties(const InterfaceState& state)
{
	// reset properties to their defaults
	properties_.reset();
	// first init from INTERFACE
	properties_.performInitFrom(Stage::INTERFACE, state.properties());
	// then init from PARENT
	properties_.performInitFrom(Stage::PARENT, parent()->properties());
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
		initProperties(state);
		if (me->computeForward(state))
			result |= true;
	}
	if ((dir & PropagatingEitherWay::BACKWARD) && hasEndState()) {
		const InterfaceState& state = fetchEndState();
		initProperties(state);
		if (me->computeBackward(state))
			result |= true;
	}
	return result;
}

void PropagatingEitherWayPrivate::newStartState(const Interface::iterator &it)
{
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if(next_start_state_ == starts_->end())
		--next_start_state_;
}

void PropagatingEitherWayPrivate::newEndState(const Interface::iterator &it)
{
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if(next_end_state_ == ends_->end())
		--next_end_state_;
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
		if (!impl->starts_) { // keep existing interface if possible
			impl->starts_.reset(new Interface([impl](const Interface::iterator& it) { impl->newStartState(it); }));
			impl->next_start_state_ = impl->starts_->begin();
		}
	} else {
		impl->starts_.reset();
		impl->next_start_state_ = Interface::iterator();
	}

	if (impl->dir & PropagatingEitherWay::BACKWARD) {
		if (!impl->ends_) { // keep existing interface if possible
			impl->ends_.reset(new Interface([impl](const Interface::iterator& it) { impl->newEndState(it); }));
			impl->next_end_state_ = impl->ends_->end();
		}
	} else {
		impl->ends_.reset();
		impl->next_end_state_ = Interface::iterator();
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

void PropagatingEitherWay::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	ComputeBase::init(scene);

	auto impl = pimpl();

	// after being connected, restrict actual interface directions
	if (!impl->nextStarts()) {
		impl->starts_.reset();
		impl->next_start_state_ = Interface::iterator();
	}
	if (!impl->prevEnds()) {
		impl->ends_.reset();
		impl->next_end_state_ = Interface::iterator();
	}
	if (!impl->isConnected())
		throw InitStageException(*this, "can neither send forwards nor backwards");
}

void PropagatingEitherWay::sendForward(const InterfaceState& from,
                                       InterfaceState&& to,
                                       const robot_trajectory::RobotTrajectoryPtr& t,
                                       double cost){
	auto impl = pimpl();
	std::cout << "sending state forward" << std::endl;
	SubTrajectory &trajectory = addTrajectory(t, cost);
	trajectory.setStartState(from);
	impl->nextStarts()->add(std::move(to), &trajectory, NULL);
	impl->newSolution(trajectory);
}

void PropagatingEitherWay::sendBackward(InterfaceState&& from,
                                        const InterfaceState& to,
                                        const robot_trajectory::RobotTrajectoryPtr& t,
                                        double cost){
	auto impl = pimpl();
	std::cout << "sending state backward" << std::endl;
	SubTrajectory& trajectory = addTrajectory(t, cost);
	trajectory.setEndState(to);
	impl->prevEnds()->add(std::move(from), NULL, &trajectory);
	impl->newSolution(trajectory);
}


PropagatingForwardPrivate::PropagatingForwardPrivate(PropagatingForward *me, const std::string &name)
   : PropagatingEitherWayPrivate(me, PropagatingEitherWay::FORWARD, name)
{
	// indicate, that we don't accept new states from ends_ interface
	ends_.reset();
	next_end_state_ = Interface::iterator();
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
	next_start_state_ = Interface::iterator();
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
	initProperties();
	return static_cast<Generator*>(me_)->compute();
}

void GeneratorPrivate::initProperties()
{
	// reset properties to their defaults
	properties_.reset();
	// then init from PARENT
	properties_.performInitFrom(Stage::PARENT, parent()->properties());
}


Generator::Generator(const std::string &name)
   : ComputeBase(new GeneratorPrivate(this, name))
{}

void Generator::spawn(InterfaceState&& state, double cost)
{
	std::cout << "spawning state forwards and backwards" << std::endl;
	assert(state.incomingTrajectories().empty() &&
	       state.outgoingTrajectories().empty());

	auto impl = pimpl();
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	SubTrajectory& trajectory = addTrajectory(dummy, cost);
	impl->prevEnds()->add(InterfaceState(state), NULL, &trajectory);
	impl->nextStarts()->add(std::move(state), &trajectory, NULL);
	impl->newSolution(trajectory);
}


ConnectingPrivate::ConnectingPrivate(Connecting *me, const std::string &name)
   : ComputeBasePrivate(me, name)
{
	starts_.reset(new Interface([this](const Interface::iterator& it) { this->newStartState(it); }));
	ends_.reset(new Interface([this](const Interface::iterator& it) { this->newEndState(it); }));
	it_pairs_ = std::make_pair(starts_->begin(), ends_->begin());
}

void ConnectingPrivate::newStartState(const Interface::iterator& it)
{
	// TODO: need to handle the pairs iterator
	if(it_pairs_.first == starts_->end())
		--it_pairs_.first;
}

void ConnectingPrivate::newEndState(const Interface::iterator& it)
{
	// TODO: need to handle the pairs iterator properly
	if(it_pairs_.second == ends_->end())
		--it_pairs_.second;
}

void ConnectingPrivate::initProperties(const InterfaceState &start, const InterfaceState &end)
{
	// reset properties to their defaults
	properties_.reset();
	// then init from PARENT
	properties_.performInitFrom(Stage::PARENT, parent()->properties());
}

bool ConnectingPrivate::canCompute() const{
	// TODO: implement this properly
	return it_pairs_.first != starts_->end() &&
	       it_pairs_.second != ends_->end();
}

bool ConnectingPrivate::compute() {
	// TODO: implement this properly
	const InterfaceState& from = *it_pairs_.first;
	const InterfaceState& to = *(it_pairs_.second++);
	initProperties(from, to);
	return static_cast<Connecting*>(me_)->compute(from, to);
}


Connecting::Connecting(const std::string &name)
   : ComputeBase(new ConnectingPrivate(this, name))
{
}

void Connecting::connect(const InterfaceState& from, const InterfaceState& to,
                         const robot_trajectory::RobotTrajectoryPtr& t, double cost) {
	auto impl = pimpl();
	SubTrajectory& trajectory = addTrajectory(t, cost);
	trajectory.setStartState(from);
	trajectory.setEndState(to);
	impl->newSolution(trajectory);
}

} }
