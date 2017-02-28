#include <moveit_task_constructor/subtask.h>

moveit::task_constructor::SubTask::SubTask(std::string name)
	: name_(name)
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

void
moveit::task_constructor::SubTask::sendBothWays(robot_trajectory::RobotTrajectoryPtr& traj, planning_scene::PlanningSceneConstPtr& ps){
	trajectories_.emplace_back();
	SubTrajectory& subtraj= trajectories_.back();
	subtraj.trajectory= traj;

	subtraj.begin.reserve(predecessors_.size());
	for( SubTask* pred : predecessors_ )
		subtraj.begin.push_back( pred->newEnd(ps, &subtraj) );

	subtraj.end.resize(successors_.size());
	for( SubTaskPtr succ : successors_ )
		subtraj.end.push_back( succ->newBegin(ps, &subtraj) );
}

moveit::task_constructor::InterfaceState*
moveit::task_constructor::SubTask::newBegin(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_end){
	beginnings_.push_back( InterfaceState(ps, old_end, NULL));
	return &beginnings_.back();
}

moveit::task_constructor::InterfaceState*
moveit::task_constructor::SubTask::newEnd(planning_scene::PlanningSceneConstPtr ps, SubTrajectory* old_beginning){
	endings_.push_back( InterfaceState(ps, NULL, old_beginning));
	return &endings_.back();
}
