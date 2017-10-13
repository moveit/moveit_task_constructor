#include "subtask_p.h"
#include "container_p.h"
#include <iostream>
#include <iomanip>
#include <ros/console.h>

namespace moveit { namespace task_constructor {

SubTask::SubTask(SubTaskPrivate *impl)
   : pimpl_(impl)
{
}

SubTask::~SubTask()
{
	delete pimpl_;
}

bool SubTask::init(const planning_scene::PlanningSceneConstPtr &scene)
{
}

const std::string& SubTask::getName() const {
	return pimpl_->name_;
}

std::ostream& operator<<(std::ostream &os, const SubTask& stage) {
	os << *stage.pimpl();
	return os;
}

template<SubTaskPrivate::InterfaceFlag own, SubTaskPrivate::InterfaceFlag other>
const char* direction(const SubTaskPrivate& stage) {
	SubTaskPrivate::InterfaceFlags f = stage.deducedFlags();
	bool own_if = f & own;
	bool other_if = f & other;
	bool reverse = own & SubTaskPrivate::INPUT_IF_MASK;
	if (own_if && other_if) return "<>";
	if (!own_if && !other_if) return "--";
	if (other_if ^ reverse) return "->";
	return "<-";
};

std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage) {
	// starts
	for (const Interface* i : {stage.prev_ends_, stage.starts_.get()}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// trajectories
	os << std::setw(5) << direction<SubTaskPrivate::READS_START, SubTaskPrivate::WRITES_PREV_END>(stage)
	   << std::setw(3) << stage.trajectories_.size()
	   << std::setw(5) << direction<SubTaskPrivate::READS_END, SubTaskPrivate::WRITES_NEXT_START>(stage);
	// ends
	for (const Interface* i : {stage.ends_.get(), stage.next_starts_}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// name
	os << " / " << stage.name_;
	return os;
}


SubTaskPrivate::SubTaskPrivate(SubTask *me, const std::string &name)
   : me_(me), name_(name), parent_(nullptr), prev_ends_(nullptr), next_starts_(nullptr)
{}

SubTrajectory& SubTaskPrivate::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	trajectories_.emplace_back(trajectory);
	SubTrajectory& back = trajectories_.back();
	return back;
}

SubTaskPrivate::InterfaceFlags SubTaskPrivate::interfaceFlags() const
{
	InterfaceFlags result = announcedFlags();
	result &= ~InterfaceFlags(OWN_IF_MASK);
	result |= deducedFlags();
	return result;
}

// return the interface flags that can be deduced from the interface
inline SubTaskPrivate::InterfaceFlags SubTaskPrivate::deducedFlags() const
{
	InterfaceFlags f;
	if (starts_)  f |= READS_START;
	if (ends_) f |= READS_END;
	if (prevEnds()) f |= WRITES_PREV_END;
	if (nextStarts())  f |= WRITES_NEXT_START;
	return f;
}


PropagatingEitherWayPrivate::PropagatingEitherWayPrivate(PropagatingEitherWay *me, PropagatingEitherWay::Direction dir, const std::string &name)
   : SubTaskPrivate(me, name), dir(dir)
{
}

SubTaskPrivate::InterfaceFlags PropagatingEitherWayPrivate::announcedFlags() const {
	InterfaceFlags f;
	if (dir & PropagatingEitherWay::FORWARD)
		f |= InterfaceFlags({READS_START, WRITES_NEXT_START});
	if (dir & PropagatingEitherWay::BACKWARD)
		f |= InterfaceFlags({READS_END, WRITES_PREV_END});
	return f;
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
	planning_scene::PlanningScenePtr ps;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost;
	if ((dir & PropagatingEitherWay::FORWARD) && hasStartState()) {
		if (me->computeForward(fetchStartState()))
			result |= true;
	}
	if ((dir & PropagatingEitherWay::BACKWARD) && hasEndState()) {
		if (me->computeBackward(fetchEndState()))
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
   : SubTask(impl)
{
	initInterface();
}

PIMPL_FUNCTIONS(PropagatingEitherWay)

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

void PropagatingEitherWay::sendForward(const InterfaceState& from,
                                       const planning_scene::PlanningSceneConstPtr& to,
                                       const robot_trajectory::RobotTrajectoryPtr& t,
                                       double cost){
	auto impl = pimpl();
	std::cout << "sending state forward" << std::endl;
	SubTrajectory &trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(from);
	impl->nextStarts()->add(to, &trajectory, NULL);
}

void PropagatingEitherWay::sendBackward(const planning_scene::PlanningSceneConstPtr& from,
                                        const InterfaceState& to,
                                        const robot_trajectory::RobotTrajectoryPtr& t,
                                        double cost){
	auto impl = pimpl();
	std::cout << "sending state backward" << std::endl;
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setEndState(to);
	impl->prevEnds()->add(from, NULL, &trajectory);
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
PIMPL_FUNCTIONS(PropagatingForward)

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
PIMPL_FUNCTIONS(PropagatingBackward)

bool PropagatingBackward::computeForward(const InterfaceState &from)
{
	assert(false); // This should never be called
}


GeneratorPrivate::GeneratorPrivate(Generator *me, const std::string &name)
   : SubTaskPrivate(me, name)
{}

SubTaskPrivate::InterfaceFlags GeneratorPrivate::announcedFlags() const {
	return InterfaceFlags({WRITES_NEXT_START,WRITES_PREV_END});
}

bool GeneratorPrivate::canCompute() const {
	return static_cast<Generator*>(me_)->canCompute();
}

bool GeneratorPrivate::compute() {
	return static_cast<Generator*>(me_)->compute();
}


Generator::Generator(const std::string &name)
   : SubTask(new GeneratorPrivate(this, name))
{}
PIMPL_FUNCTIONS(Generator)

void Generator::spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost)
{
	std::cout << "spawning state forwards and backwards" << std::endl;
	auto impl = pimpl();
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	SubTrajectory& trajectory = impl->addTrajectory(dummy, cost);
	impl->prevEnds()->add(ps, NULL, &trajectory);
	impl->nextStarts()->add(ps, &trajectory, NULL);
}


ConnectingPrivate::ConnectingPrivate(Connecting *me, const std::string &name)
   : SubTaskPrivate(me, name)
{
	starts_.reset(new Interface([this](const Interface::iterator& it) { this->newStartState(it); }));
	ends_.reset(new Interface([this](const Interface::iterator& it) { this->newEndState(it); }));
	it_pairs_ = std::make_pair(starts_->begin(), ends_->begin());
}

SubTaskPrivate::InterfaceFlags ConnectingPrivate::announcedFlags() const {
	return InterfaceFlags({READS_START, READS_END});
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

bool ConnectingPrivate::canCompute() const{
	// TODO: implement this properly
	return it_pairs_.first != starts_->end() &&
	       it_pairs_.second != ends_->end();
}

bool ConnectingPrivate::compute() {
	// TODO: implement this properly
	const InterfaceState& from = *it_pairs_.first;
	const InterfaceState& to = *(it_pairs_.second++);
	return static_cast<Connecting*>(me_)->compute(from, to);
}


Connecting::Connecting(const std::string &name)
   : SubTask(new ConnectingPrivate(this, name))
{
}
PIMPL_FUNCTIONS(Connecting)

void Connecting::connect(const InterfaceState& from, const InterfaceState& to,
                         const robot_trajectory::RobotTrajectoryPtr& t, double cost) {
	auto impl = pimpl();
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(from);
	trajectory.setEndState(to);
}

} }
