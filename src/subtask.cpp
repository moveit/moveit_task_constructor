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

SubTask::InterfaceFlags SubTask::interfaceFlags() const
{
	SubTask::InterfaceFlags result = pimpl_->announcedFlags();
	result &= ~SubTask::InterfaceFlags(SubTask::OWN_IF_MASK);
	result |= pimpl_->deducedFlags();
	return result;
}

SubTask::~SubTask()
{
	delete pimpl_;
}

const std::string& SubTask::getName() const {
	return pimpl_->name_;
}

std::ostream& operator<<(std::ostream &os, const SubTask& stage) {
	os << *stage.pimpl_func();
	return os;
}

template<SubTask::InterfaceFlag own, SubTask::InterfaceFlag other>
const char* direction(const SubTaskPrivate& stage) {
	SubTask::InterfaceFlags f = stage.deducedFlags();
	bool own_if = f & own;
	bool other_if = f & other;
	bool reverse = own & SubTask::INPUT_IF_MASK;
	if (own_if && other_if) return "<>";
	if (!own_if && !other_if) return "--";
	if (other_if ^ reverse) return "->";
	return "<-";
};

std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage) {
	// inputs
	for (const Interface* i : {stage.prev_output_, stage.input_.get()}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// trajectories
	os << std::setw(5) << direction<SubTask::READS_INPUT, SubTask::WRITES_PREV_OUTPUT>(stage)
	   << std::setw(3) << stage.trajectories_.size()
	   << std::setw(5) << direction<SubTask::READS_OUTPUT, SubTask::WRITES_NEXT_INPUT>(stage);
	// outputs
	for (const Interface* i : {stage.output_.get(), stage.next_input_}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// name
	os << " / " << stage.name_;
	return os;
}

planning_scene::PlanningSceneConstPtr SubTask::scene() const
{
	return pimpl_->scene_;
}

planning_pipeline::PlanningPipelinePtr SubTask::planner() const
{
	return pimpl_->planner_;
}

void SubTask::setPlanningScene(const planning_scene::PlanningSceneConstPtr &scene){
	pimpl_->scene_= scene;
}

void SubTask::setPlanningPipeline(const planning_pipeline::PlanningPipelinePtr &planner){
	pimpl_->planner_= planner;
}


SubTaskPrivate::SubTaskPrivate(SubTask *me, const std::__cxx11::string &name)
   : me_(me), name_(name), parent_(nullptr), prev_output_(nullptr), next_input_(nullptr)
{}

SubTrajectory& SubTaskPrivate::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

// return the interface flags that can be deduced from the interface
inline SubTask::InterfaceFlags SubTaskPrivate::deducedFlags() const
{
	SubTask::InterfaceFlags f;
	if (input_)  f |= SubTask::READS_INPUT;
	if (output_) f |= SubTask::READS_OUTPUT;
	if (prevOutput()) f |= SubTask::WRITES_PREV_OUTPUT;
	if (nextInput())  f |= SubTask::WRITES_NEXT_INPUT;
	return f;
}

inline bool SubTaskPrivate::sendForward(SubTrajectory &trajectory, const planning_scene::PlanningSceneConstPtr &ps){
	std::cout << "sending state to start" << std::endl;
	if (next_input_) {
		next_input_->add(ps, &trajectory, NULL);
		return true;
	}
	return false;
}

inline bool SubTaskPrivate::sendBackward(SubTrajectory &trajectory, const planning_scene::PlanningSceneConstPtr &ps){
	std::cout << "sending state to end" << std::endl;
	if (prev_output_) {
		prev_output_->add(ps, NULL, &trajectory);
		return true;
	}
	return false;
}


PropagatingAnyWayPrivate::PropagatingAnyWayPrivate(PropagatingAnyWay *me, PropagatingAnyWay::Direction dir, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name), dir(dir)
{
}

SubTask::InterfaceFlags PropagatingAnyWayPrivate::announcedFlags() const {
	SubTask::InterfaceFlags f;
	if (dir & PropagatingAnyWay::FORWARD)
		f |= SubTask::InterfaceFlags({SubTask::READS_INPUT, SubTask::WRITES_NEXT_INPUT});
	if (dir & PropagatingAnyWay::BACKWARD)
		f |= SubTask::InterfaceFlags({SubTask::READS_OUTPUT, SubTask::WRITES_PREV_OUTPUT});
	return f;
}

inline bool PropagatingAnyWayPrivate::hasStartState() const{
	return next_input_state_ != input_->end();
}

const InterfaceState& PropagatingAnyWayPrivate::fetchStartState(){
	if (!hasStartState())
		throw std::runtime_error("no new state for beginning available");

	const InterfaceState& state= *next_input_state_;
	++next_input_state_;

	return state;
}

bool PropagatingAnyWayPrivate::sendForward(const robot_trajectory::RobotTrajectoryPtr& t,
                                           const InterfaceState& from,
                                           const planning_scene::PlanningSceneConstPtr& to,
                                           double cost){
	SubTrajectory &trajectory = addTrajectory(t, cost);
	trajectory.setStartState(from);
	return SubTaskPrivate::sendForward(trajectory, to);
}


inline bool PropagatingAnyWayPrivate::hasEndState() const{
	return next_output_state_ != output_->end();
}

const InterfaceState& PropagatingAnyWayPrivate::fetchEndState(){
	if(!hasEndState())
		throw std::runtime_error("no new state for ending available");

	const InterfaceState& state= *next_output_state_;
	++next_output_state_;

	return state;
}

bool PropagatingAnyWayPrivate::sendBackward(const robot_trajectory::RobotTrajectoryPtr& t,
                                            const planning_scene::PlanningSceneConstPtr& from,
                                            const InterfaceState& to,
                                            double cost){
	SubTrajectory& trajectory = addTrajectory(t, cost);
	trajectory.setEndState(to);
	return SubTaskPrivate::sendBackward(trajectory, from);
}


PropagatingAnyWay::PropagatingAnyWay(const std::string &name)
   : PropagatingAnyWay(new PropagatingAnyWayPrivate(this, ANYWAY, name))
{
}

PropagatingAnyWay::PropagatingAnyWay(PropagatingAnyWayPrivate *impl)
   : SubTask(impl)
{
	initInterface();
}

void PropagatingAnyWay::initInterface()
{
	IMPL(PropagatingAnyWay)
	if (impl->dir & PropagatingAnyWay::FORWARD) {
		if (!impl->input_) { // keep existing interface if possible
			impl->input_.reset(new Interface([this](const Interface::iterator& it) { newInputState(it); }));
			impl->next_input_state_ = impl->input_->begin();
		}
	} else {
		impl->input_.reset();
		impl->next_input_state_ = Interface::iterator();
	}

	if (impl->dir & PropagatingAnyWay::BACKWARD) {
		if (!impl->output_) { // keep existing interface if possible
			impl->output_.reset(new Interface([this](const Interface::iterator& it) { newOutputState(it); }));
			impl->next_output_state_ = impl->output_->end();
		}
	} else {
		impl->output_.reset();
		impl->next_output_state_ = Interface::iterator();
	}
}

void PropagatingAnyWay::restrictDirection(PropagatingAnyWay::Direction dir)
{
	IMPL(PropagatingAnyWay);
	if (impl->dir == dir) return;
	if (impl->isConnected())
		throw std::runtime_error("Cannot change direction after being connected");
	impl->dir = dir;
	initInterface();
}

bool PropagatingAnyWay::computeForward(const InterfaceState &from, planning_scene::PlanningScenePtr &to,
                                       robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost)
{
	// default implementation is void, needs override by user
	return false;
}

bool PropagatingAnyWay::computeBackward(planning_scene::PlanningScenePtr &from, const InterfaceState &to,
                                        robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost)
{
	// default implementation is void, needs override by user
	return false;
}

bool PropagatingAnyWay::canCompute() const
{
	IMPL(const PropagatingAnyWay);
	if ((impl->dir & PropagatingAnyWay::FORWARD) && impl->hasStartState())
		return true;
	if ((impl->dir & PropagatingAnyWay::BACKWARD) && impl->hasEndState())
		return true;
	return false;
}

bool PropagatingAnyWay::compute()
{
	IMPL(PropagatingAnyWay);
	bool result = false;
	planning_scene::PlanningScenePtr ps;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost;
	if ((impl->dir & PropagatingAnyWay::FORWARD) && impl->hasStartState()) {
		const InterfaceState& from = impl->fetchStartState();
		if (computeForward(from, ps, trajectory, cost) && ps
		    && impl->sendForward(trajectory, from, ps, cost))
			result |= true;
	}
	if ((impl->dir & PropagatingAnyWay::BACKWARD) && impl->hasEndState()) {
		const InterfaceState& to = impl->fetchEndState();
		if (computeBackward(ps, to, trajectory, cost) && ps
		    && impl->sendBackward(trajectory, ps, to, cost))
			result |= true;
	}
	return result;
}

void PropagatingAnyWay::newInputState(const Interface::iterator &it)
{
	IMPL(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if(impl->next_input_state_ == impl->input_->end())
		--impl->next_input_state_;
}

void PropagatingAnyWay::newOutputState(const Interface::iterator &it)
{
	IMPL(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( impl->next_output_state_ == impl->output_->end() )
		--impl->next_output_state_;
}


PropagatingForwardPrivate::PropagatingForwardPrivate(PropagatingForward *me, const std::__cxx11::string &name)
   : PropagatingAnyWayPrivate(me, PropagatingAnyWay::FORWARD, name)
{
	// indicate, that we don't accept new states from output interface
	output_.reset();
	next_output_state_ = Interface::iterator();
}


PropagatingForward::PropagatingForward(const std::string& name)
   : PropagatingAnyWay(new PropagatingForwardPrivate(this, name))
{}


PropagatingBackwardPrivate::PropagatingBackwardPrivate(PropagatingBackward *me, const std::__cxx11::string &name)
   : PropagatingAnyWayPrivate(me, PropagatingAnyWay::BACKWARD, name)
{
	// indicate, that we don't accept new states from input interface
	input_.reset();
	next_input_state_ = Interface::iterator();
}


PropagatingBackward::PropagatingBackward(const std::string &name)
   : PropagatingAnyWay(new PropagatingBackwardPrivate(this, name))
{}


GeneratorPrivate::GeneratorPrivate(Generator *me, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name)
{}

SubTask::InterfaceFlags GeneratorPrivate::announcedFlags() const {
	return SubTask::InterfaceFlags({SubTask::WRITES_NEXT_INPUT,SubTask::WRITES_PREV_OUTPUT});
}

inline bool GeneratorPrivate::spawn(const planning_scene::PlanningSceneConstPtr &ps, double cost)
{
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	SubTrajectory& trajectory = addTrajectory(dummy, cost);

	std::cout << "spawning state" << std::endl;
	bool result = sendBackward(trajectory, ps);
	result |= sendForward(trajectory, ps);
	return result;
}


Generator::Generator(const std::string &name)
   : SubTask(new GeneratorPrivate(this, name))
{}

bool Generator::spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost)
{
	IMPL(Generator)
	return impl->spawn(ps, cost);
}


ConnectingPrivate::ConnectingPrivate(Connecting *me, const std::__cxx11::string &name)
   : SubTaskPrivate(me, name)
{
	input_.reset(new Interface(std::bind(&Connecting::newInputState, me, std::placeholders::_1)));
	output_.reset(new Interface(std::bind(&Connecting::newOutputState, me, std::placeholders::_1)));
	it_pairs_ = std::make_pair(input_->begin(), output_->begin());
}

SubTask::InterfaceFlags ConnectingPrivate::announcedFlags() const {
	return SubTask::InterfaceFlags({SubTask::READS_INPUT, SubTask::READS_OUTPUT});
}

inline void ConnectingPrivate::connect(const robot_trajectory::RobotTrajectoryPtr &t, const InterfaceStatePair &state_pair, double cost)
{
	SubTrajectory& trajectory = addTrajectory(t, cost);
	trajectory.setStartState(state_pair.first);
	trajectory.setEndState(state_pair.second);
}


Connecting::Connecting(const std::string &name)
   : SubTask(new ConnectingPrivate(this, name))
{
}

void Connecting::newInputState(const Interface::iterator& it)
{
	IMPL(Connecting);
	// TODO: need to handle the pairs iterator
	if( impl->it_pairs_.first == impl->input_->end() )
		--impl->it_pairs_.first;
}

void Connecting::newOutputState(const Interface::iterator& it)
{
	IMPL(Connecting);
	// TODO: need to handle the pairs iterator properly
	if( impl->it_pairs_.second == impl->output_->end() )
		--impl->it_pairs_.second;
}

bool Connecting::hasStatePair() const{
	IMPL(const Connecting);
	// TODO: implement this properly
	return impl->it_pairs_.first != impl->input_->end() &&
	       impl->it_pairs_.second != impl->output_->end();
}

InterfaceStatePair Connecting::fetchStatePair(){
	IMPL(Connecting);
	// TODO: implement this properly
	return std::pair<const InterfaceState&, const InterfaceState&>
	      (*impl->it_pairs_.first, *(impl->it_pairs_.second++));
}

void Connecting::connect(const robot_trajectory::RobotTrajectoryPtr& t, const InterfaceStatePair& state_pair, double cost)
{
	IMPL(Connecting)
	impl->connect(t, state_pair, cost);
}

} }
