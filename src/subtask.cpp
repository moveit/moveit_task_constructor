#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace task_constructor {

SubTask::SubTask(std::string name)
	: name_(name),
	  it_beginnings_(beginnings_.begin()),
	  it_endings_(endings_.begin()),
	  it_pairs_(beginnings_.begin(), endings_.begin())
{}

void SubTask::addPredecessor(SubTaskPtr prev_task){
	predecessor_= SubTaskWeakPtr(prev_task);
}

void SubTask::addSuccessor(SubTaskPtr next_task){
	successor_= next_task;
}

const std::string&
SubTask::getName(){
	return name_;
}

const std::list<InterfaceState>&
SubTask::getBeginning(){
	return beginnings_;
}

const std::list<InterfaceState>&
SubTask::getEnd(){
	return endings_;
}

std::list<SubTrajectory>&
SubTask::getTrajectories(){
	return trajectories_;
}

void
SubTask::setPlanningScene(planning_scene::PlanningSceneConstPtr scene){
	scene_= scene;
}

void
SubTask::setPlanningPipeline(planning_pipeline::PlanningPipelinePtr planner){
	planner_= planner;
}

InterfaceState&
SubTask::fetchStateBeginning(){
	if(it_beginnings_ == beginnings_.end())
		throw std::runtime_error("no new state for beginning available");

	InterfaceState& state= *it_beginnings_;
	++it_beginnings_;

	return state;
}

InterfaceState&
SubTask::fetchStateEnding(){
	if(it_endings_ == endings_.end())
		throw std::runtime_error("no new state for ending available");

	InterfaceState& state= *it_endings_;
	++it_endings_;

	return state;
}

std::pair<InterfaceState&, InterfaceState&>
SubTask::fetchStatePair(){
	// TODO: implement this properly
	return std::pair<InterfaceState&, InterfaceState&>(
		*it_pairs_.first,
		*(it_pairs_.second++));
}

SubTrajectory&
SubTask::addTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

void
SubTask::sendForward(SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	if( successor_ ){
		std::cout << "sending state forward to successor" << std::endl;
		traj.end= successor_->newBeginning(ps, &traj);
	}
}

void
SubTask::sendBackward(SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	if( !predecessor_.expired() ){
		std::cout << "sending state backward to predecessor" << std::endl;
		traj.begin= predecessor_.lock()->newEnd(ps, &traj);
	}
}

void
SubTask::sendBothWays(SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	std::cout << "sending state both ways" << std::endl;
	if( !predecessor_.expired() )
		sendBackward(traj, ps);
	if( successor_ )
		sendForward(traj, ps);
}

InterfaceState*
SubTask::newBeginning(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_end){
	assert( bool(ps) );

	beginnings_.push_back( InterfaceState(ps, old_end, NULL) );

	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( it_beginnings_ == beginnings_.end() )
		--it_beginnings_;

	// TODO: need to handle the pairs iterator
	if( it_pairs_.first == beginnings_.end() )
		--it_pairs_.first;

	return &beginnings_.back();
}

InterfaceState*
SubTask::newEnd(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_beginning){
	assert( bool(ps) );
	endings_.push_back( InterfaceState(ps, NULL, old_beginning) );

	// we just appended a state to the list, but the iterator doesn't see it anymore
	// so let's point it at the new one
	if( it_endings_ == endings_.end() )
		--it_endings_;

	//TODO: need to handle the pairs iterator properly
	if( it_pairs_.second == endings_.end() )
		--it_pairs_.second;

	return &endings_.back();
}

bool
SubTask::hasBeginning(){
	return it_beginnings_ != beginnings_.end();
}

bool
SubTask::hasEnding(){
	return it_endings_ != endings_.end();
}

bool
SubTask::hasStatePair(){
	// TODO: implement this properly
	return it_pairs_.first != beginnings_.end() && it_pairs_.second != endings_.end();
}

} }
