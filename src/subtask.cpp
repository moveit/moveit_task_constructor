#include <moveit_task_constructor/subtask.h>

moveit::task_constructor::SubTask::SubTask(std::string name)
	: name_(name),
	  ins_(new std::vector<InterfaceState>),
	  outs_(new std::vector<InterfaceState>)
{};

void moveit::task_constructor::SubTask::addPredecessor(SubTaskConstPtr prev_task){
	predecessors_.push_back( prev_task.get() );
}

void moveit::task_constructor::SubTask::addSuccessor(SubTaskConstPtr next_task){
	successors_.push_back( next_task );
}

const std::vector<moveit::task_constructor::InterfaceState>&
moveit::task_constructor::SubTask::getIn(){
	return *ins_;
}

const std::vector<moveit::task_constructor::InterfaceState>&
moveit::task_constructor::SubTask::getOut(){
	return *outs_;
}

const std::list<moveit::task_constructor::SubTrajectory>&
moveit::task_constructor::SubTask::getTrajectories(){
	return trajectories_;
}
