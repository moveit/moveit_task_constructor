#include <moveit_task_constructor/subtask.h>

moveit::task_constructor::SubTask::SubTask(std::string name)
	: name_(name),
	  it_beginnings_(beginnings_.begin()),
	  it_endings_(endings_.begin())
{};

void moveit::task_constructor::SubTask::addPredecessor(SubTaskPtr prev_task){
	predecessors_.push_back( prev_task.get() );
}

void moveit::task_constructor::SubTask::addSuccessor(SubTaskPtr next_task){
	successors_.push_back( next_task );
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

const std::list<moveit::task_constructor::SubTrajectory>&
moveit::task_constructor::SubTask::getTrajectories(){
	return trajectories_;
}

void
moveit::task_constructor::SubTask::setPlanningScene(planning_scene::PlanningSceneConstPtr scene){
	scene_= scene;
}

moveit::task_constructor::InterfaceState&
moveit::task_constructor::SubTask::fetchStateBeginning(){
	if(it_beginnings_ == beginnings_.end())
		throw std::runtime_error("no new state for beginning available");

	moveit::task_constructor::InterfaceState& state= *it_beginnings_;
	++it_beginnings_;

	return state;
}

moveit::task_constructor::SubTrajectory&
moveit::task_constructor::SubTask::addTrajectory(robot_trajectory::RobotTrajectoryPtr trajectory){
	trajectories_.emplace_back(trajectory);
	return trajectories_.back();
}

void
moveit::task_constructor::SubTask::sendForward(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	std::cout << "sending state forward to " << successors_.size() << " successors" << std::endl;
	traj.end.reserve(successors_.size());
	for( SubTaskPtr succ : successors_ )
		traj.end.push_back( succ->newBegin(ps, &traj) );
}

void
moveit::task_constructor::SubTask::sendBackward(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	std::cout << "sending state backward to " << predecessors_.size() << " successors" << std::endl;
	traj.begin.reserve(successors_.size());
	for( SubTask* pred : predecessors_ )
		traj.begin.push_back( pred->newEnd(ps, &traj) );
}

void
moveit::task_constructor::SubTask::sendBothWays(moveit::task_constructor::SubTrajectory& traj, planning_scene::PlanningSceneConstPtr ps){
	std::cout << "sending state both ways" << std::endl;
	if( predecessors_.size() > 0 )
		sendBackward(traj, ps);
	if( successors_.size() > 0 )
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
