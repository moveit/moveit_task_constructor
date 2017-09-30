#include "subtask_p.h"

// get correctly casted private impl pointer
#define I(Class) Class##Private * const impl = reinterpret_cast<Class##Private*>(impl_)

namespace moveit { namespace task_constructor {

SubTask::SubTask(SubTaskPrivate *impl)
   : impl_(impl)
{
}

SubTask::~SubTask()
{
	delete impl_;
}

const std::string& SubTask::getName() const {
	return impl_->name_;
}

void SubTask::setPlanningScene(planning_scene::PlanningSceneConstPtr scene){
	scene_= scene;
}

void SubTask::setPlanningPipeline(planning_pipeline::PlanningPipelinePtr planner){
	planner_= planner;
}


void SubTaskPrivate::addPredecessor(SubTaskPtr prev_task){
	predecessor_= SubTaskWeakPtr(prev_task);
}

void SubTaskPrivate::addSuccessor(SubTaskPtr next_task){
	successor_= next_task;
}

SubTrajectory& SubTaskPrivate::addTrajectory(const robot_trajectory::RobotTrajectoryPtr& trajectory, double cost){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

void SubTaskPrivate::sendForward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps){
	if( successor_ ){
		std::cout << "sending state forward to successor" << std::endl;
		traj.end= successor_->impl_->newBeginning(ps, &traj);
	}
}

void SubTaskPrivate::sendBackward(SubTrajectory& traj, const planning_scene::PlanningSceneConstPtr& ps){
	if( !predecessor_.expired() ){
		std::cout << "sending state backward to predecessor" << std::endl;
		traj.begin= predecessor_.lock()->impl_->newEnd(ps, &traj);
	}
}

InterfaceState* SubTaskPrivate::newBeginning(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_end){
	assert( bool(ps) );
	beginnings_.push_back( InterfaceState(ps, old_end, NULL) );

	me_->newBeginning(--beginnings_.end());
	return &beginnings_.back();
}

InterfaceState* SubTaskPrivate::newEnd(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_beginning){
	assert( bool(ps) );
	endings_.push_back( InterfaceState(ps, NULL, old_beginning) );

	me_->newEnd(--endings_.end());
	return &endings_.back();
}


PropagatingAnyWay::PropagatingAnyWay(const std::string &name)
   : SubTask(new PropagatingAnyWayPrivate(this, name))
{}

PropagatingAnyWay::PropagatingAnyWay(PropagatingAnyWayPrivate *impl) : SubTask(impl) {}

bool PropagatingAnyWay::hasBeginning() const{
	I(PropagatingAnyWay);
	return impl->it_beginnings_ != impl->beginnings_.end();
}

InterfaceState& PropagatingAnyWay::fetchStateBeginning(){
	I(PropagatingAnyWay);
	if(impl->it_beginnings_ == impl->beginnings_.end())
		throw std::runtime_error("no new state for beginning available");

	InterfaceState& state= *impl->it_beginnings_;
	++impl->it_beginnings_;

	return state;
}

void PropagatingAnyWay::sendForward(const robot_trajectory::RobotTrajectoryPtr& t,
                                    const InterfaceState& from,
                                    const planning_scene::PlanningSceneConstPtr& to,
                                    double cost){
	SubTrajectory &trajectory = impl_->addTrajectory(t, cost);
	trajectory.setBeginning(from);
	impl_->sendForward(trajectory, to);
}

void PropagatingAnyWay::newBeginning(const InterfaceStateList::iterator &it)
{
	I(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( impl->it_beginnings_ == impl->beginnings_.end() )
		--impl->it_beginnings_;
}

bool PropagatingAnyWay::hasEnding() const{
	I(PropagatingAnyWay);
	return impl->it_endings_ != impl->endings_.end();
}

InterfaceState& PropagatingAnyWay::fetchStateEnding(){
	I(PropagatingAnyWay);
	if(impl->it_endings_ == impl->endings_.end())
		throw std::runtime_error("no new state for ending available");

	InterfaceState& state= *impl->it_endings_;
	++impl->it_endings_;

	return state;
}

void PropagatingAnyWay::sendBackward(const robot_trajectory::RobotTrajectoryPtr& t,
                                       const planning_scene::PlanningSceneConstPtr& from,
                                       const InterfaceState& to,
                                       double cost){
   SubTrajectory& trajectory = impl_->addTrajectory(t, cost);
   trajectory.setEnding(to);
	impl_->sendBackward(trajectory, from);
}

void PropagatingAnyWay::newEnd(const InterfaceStateList::iterator &it)
{
	I(PropagatingAnyWay);
	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( impl->it_endings_ == impl->endings_.end() )
		--impl->it_endings_;
}


PropagatingForward::PropagatingForward(const std::string& name)
   : PropagatingAnyWay(new PropagatingForwardPrivate(this, name))
{}


PropagatingBackward::PropagatingBackward(const std::string &name)
   : PropagatingAnyWay(new PropagatingBackwardPrivate(this, name))
{}


Generator::Generator(const std::string &name)
   : SubTask(new SubTaskPrivate(this, name))
{}

void Generator::spawn(const planning_scene::PlanningSceneConstPtr& ps, double cost)
{
	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr dummy;
	moveit::task_constructor::SubTrajectory& trajectory = impl_->addTrajectory(dummy, cost);

	std::cout << "spawning state" << std::endl;
	impl_->sendBackward(trajectory, ps);
	impl_->sendForward(trajectory, ps);
}


Connecting::Connecting(const std::string &name)
   : SubTask(new ConnectingPrivate(this, name))
{
}

void Connecting::newBeginning(const InterfaceStateList::iterator& it)
{
	I(Connecting);
	// TODO: need to handle the pairs iterator
	if( impl->it_pairs_.first == impl->beginnings_.end() )
		--impl->it_pairs_.first;
}

void Connecting::newEnd(const InterfaceStateList::iterator& it)
{
	I(Connecting);
	// TODO: need to handle the pairs iterator properly
	if( impl->it_pairs_.second == impl->endings_.end() )
		--impl->it_pairs_.second;
}

bool Connecting::hasStatePair() const{
	I(Connecting);
	// TODO: implement this properly
	return impl->it_pairs_.first != impl->beginnings_.end() &&
	       impl->it_pairs_.second != impl->endings_.end();
}

std::pair<InterfaceState&, InterfaceState&> Connecting::fetchStatePair(){
	I(Connecting);
	// TODO: implement this properly
	return std::pair<InterfaceState&, InterfaceState&>
	      (*impl->it_pairs_.first, *(impl->it_pairs_.second++));
}

void Connecting::connect(const robot_trajectory::RobotTrajectoryPtr& t, const InterfaceStatePair& state_pair, double cost)
{
	moveit::task_constructor::SubTrajectory& trajectory = impl_->addTrajectory(t, cost);
	trajectory.setBeginning(state_pair.first);
	trajectory.setEnding(state_pair.second);
}


} }
