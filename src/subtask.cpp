#include <moveit_task_constructor/subtask.h>

moveit::task_constructor::SubTask::SubTask(std::string name)
	: name_(name),
	  predecessor_(NULL),
	  it_beginnings_(beginnings_.begin()),
	  it_endings_(endings_.begin()),
	  it_pairs_(beginnings_.begin(), endings_.begin())
{};

void moveit::task_constructor::SubTask::addPredecessor(SubTaskPtr prev_task){
	predecessor_= prev_task.get();
}

void moveit::task_constructor::SubTask::addSuccessor(SubTaskPtr next_task){
	successor_= next_task;
}

const std::string&
moveit::task_constructor::SubTask::getName(){
	return name_;
}

const std::list<moveit::task_constructor::InterfaceState>&
moveit::task_constructor::SubTask::getBegin(){
	return beginnings_;
}

const std::list<moveit::task_constructor::InterfaceState>&
moveit::task_constructor::SubTask::getEnd(){
	return endings_;
}

std::list<moveit::task_constructor::SubTrajectory>&
moveit::task_constructor::SubTask::getTrajectories(){
	return trajectories_;
}

void
moveit::task_constructor::SubTask::setPlanningScene(planning_scene::PlanningSceneConstPtr scene){
	scene_= scene;
}

void
moveit::task_constructor::SubTask::setPlanningPipeline(planning_pipeline::PlanningPipelinePtr planner){
	planner_= planner;
}

moveit::task_constructor::InterfaceState&
moveit::task_constructor::SubTask::fetchStateBeginning(){
	if(it_beginnings_ == beginnings_.end())
		throw std::runtime_error("no new state for beginning available");

	moveit::task_constructor::InterfaceState& state= *it_beginnings_;
	++it_beginnings_;

	return state;
}

moveit::task_constructor::InterfaceState&
moveit::task_constructor::SubTask::fetchStateEnding(){
	if(it_endings_ == endings_.end())
		throw std::runtime_error("no new state for ending available");

	moveit::task_constructor::InterfaceState& state= *it_endings_;
	++it_endings_;

	return state;
}

std::pair<moveit::task_constructor::InterfaceState&, moveit::task_constructor::InterfaceState&>
moveit::task_constructor::SubTask::fetchStatePair(){
	// TODO: implement this properly
	return std::pair<moveit::task_constructor::InterfaceState&, moveit::task_constructor::InterfaceState&>(
		*it_pairs_.first,
		*(it_pairs_.second++));
}

moveit::task_constructor::SubTrajectory&
moveit::task_constructor::SubTask::addTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

void
moveit::task_constructor::SubTask::sendForward(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	if( successor_ ){
		std::cout << "sending state forward to successor" << std::endl;
		traj.end= successor_->newBegin(ps, &traj);
	}
}

void
moveit::task_constructor::SubTask::sendBackward(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	if( predecessor_ != NULL ){
		std::cout << "sending state backward to predecessor" << std::endl;
		traj.begin= predecessor_->newEnd(ps, &traj);
	}
}

void
moveit::task_constructor::SubTask::sendBothWays(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	std::cout << "sending state both ways" << std::endl;
	if( predecessor_ != NULL )
		sendBackward(traj, ps);
	if( successor_ )
		sendForward(traj, ps);
}

moveit::task_constructor::InterfaceState*
moveit::task_constructor::SubTask::newBegin(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_end){
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

moveit::task_constructor::InterfaceState*
moveit::task_constructor::SubTask::newEnd(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_beginning){
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
moveit::task_constructor::SubTask::hasBeginning(){
	return it_beginnings_ != beginnings_.end();
}

bool
moveit::task_constructor::SubTask::hasEnding(){
	return it_endings_ != endings_.end();
}

bool
moveit::task_constructor::SubTask::hasStatePair(){
	// TODO: implement this properly
	return it_pairs_.first != beginnings_.end() && it_pairs_.second != endings_.end();
}
