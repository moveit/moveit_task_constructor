#include "subtask_p.h"
#include "container_p.h"
#include <iostream>
#include <iomanip>

namespace moveit { namespace task_constructor {

SubTask::SubTask(SubTaskPrivate *impl)
   : pimpl_(impl)
{
}

SubTask::InterfaceFlags SubTask::interfaceFlags() const
{
	SubTask::InterfaceFlags result = pimpl_->interfaceFlags();
	result &= ~SubTask::InterfaceFlags(SubTask::READS_MASK);
	result |= pimpl_->SubTaskPrivate::interfaceFlags();
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
std::ostream& operator<<(std::ostream &os, const SubTaskPrivate& stage) {
	// inputs
	for (const InterfacePtr &i : {stage.prevOutput(), stage.input_}) {
		os << std::setw(3);
		if (i) os << i->size();
		else os << "-";
	}
	// trajectories
	os << " -> " << std::setw(3) << stage.trajectories_.size() << " <- ";
	// outputs
	for (const InterfacePtr &i : {stage.output_, stage.nextInput()}) {
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

SubTrajectory& SubTaskPrivate::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

SubTask::InterfaceFlags SubTaskPrivate::interfaceFlags() const
{
	// the base class provides the read flags
	SubTask::InterfaceFlags f;
	if (input_)  f |= SubTask::READS_INPUT;
	if (output_) f |= SubTask::READS_OUTPUT;
	return f;
}

void SubTaskPrivate::sendForward(SubTrajectory& trajectory, const planning_scene::PlanningSceneConstPtr& ps){
	std::cout << "sending state to start" << std::endl;
	nextInput()->add(ps, &trajectory, NULL);
}

void SubTaskPrivate::sendBackward(SubTrajectory& trajectory, const planning_scene::PlanningSceneConstPtr& ps){
	std::cout << "sending state to end" << std::endl;
	prevOutput()->add(ps, NULL, &trajectory);
}

const SubTaskPrivate* SubTaskPrivate::prev() const {
	return parent_ ? parent_->prev_(this) : nullptr;
}
const SubTaskPrivate* SubTaskPrivate::next() const {
	return parent_ ? parent_->next_(this) : nullptr;
}


PropagatingAnyWay::PropagatingAnyWay(const std::string &name)
   : SubTask(new PropagatingAnyWayPrivate(this, name))
{}

PropagatingAnyWay::PropagatingAnyWay(PropagatingAnyWayPrivate *impl) : SubTask(impl) {}

bool PropagatingAnyWay::hasBeginning() const{
	IMPL(const PropagatingAnyWay);
	return impl->next_input_ != impl->input_->end();
}

const InterfaceState& PropagatingAnyWay::fetchStateBeginning(){
	IMPL(PropagatingAnyWay);
	if (!hasBeginning())
		throw std::runtime_error("no new state for beginning available");

	const InterfaceState& state= *impl->next_input_;
	++impl->next_input_;

	return state;
}

void PropagatingAnyWay::sendForward(const robot_trajectory::RobotTrajectoryPtr& t,
                                    const InterfaceState& from,
                                    const planning_scene::PlanningSceneConstPtr& to,
                                    double cost){
	IMPL(PropagatingAnyWay);
	SubTrajectory &trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(from);
	impl->sendForward(trajectory, to);
}

void PropagatingAnyWay::newInputState(const Interface::iterator &it)
{
	IMPL(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( impl->next_input_ == impl->input_->end() )
		--impl->next_input_;
}

bool PropagatingAnyWay::hasEnding() const{
	IMPL(const PropagatingAnyWay);
	return impl->next_output_ != impl->output_->end();
}

const InterfaceState& PropagatingAnyWay::fetchStateEnding(){
	IMPL(PropagatingAnyWay);
	if(!hasEnding())
		throw std::runtime_error("no new state for ending available");

	const InterfaceState& state= *impl->next_output_;
	++impl->next_output_;

	return state;
}

void PropagatingAnyWay::sendBackward(const robot_trajectory::RobotTrajectoryPtr& t,
                                     const planning_scene::PlanningSceneConstPtr& from,
                                     const InterfaceState& to,
                                     double cost){
	IMPL(PropagatingAnyWay);
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setEndState(to);
	impl->sendBackward(trajectory, from);
}

void PropagatingAnyWay::newOutputState(const Interface::iterator &it)
{
	IMPL(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( impl->next_output_ == impl->output_->end() )
		--impl->next_output_;
}


PropagatingForward::PropagatingForward(const std::string& name)
   : PropagatingAnyWay(new PropagatingForwardPrivate(this, name))
{}


PropagatingBackward::PropagatingBackward(const std::string &name)
   : PropagatingAnyWay(new PropagatingBackwardPrivate(this, name))
{}


Generator::Generator(const std::string &name)
   : SubTask(new GeneratorPrivate(this, name))
{}

void Generator::spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost)
{
	IMPL(Generator);
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	SubTrajectory& trajectory = impl->addTrajectory(dummy, cost);

	std::cout << "spawning state" << std::endl;
	impl->sendBackward(trajectory, ps);
	impl->sendForward(trajectory, ps);
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
	IMPL(Connecting);
	SubTrajectory& trajectory = impl->addTrajectory(t, cost);
	trajectory.setStartState(state_pair.first);
	trajectory.setEndState(state_pair.second);
}


} }
