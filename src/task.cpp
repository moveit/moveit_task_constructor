#include <moveit_task_constructor/task.h>
#include <moveit_task_constructor/subtask.h>

moveit::task_constructor::Task::Task()
{}

void moveit::task_constructor::Task::addStart( SubTaskPtr subtask ){
	subtasks_.clear();
	subtasks_.push_back( subtask );
}

void moveit::task_constructor::Task::addAfter( SubTaskPtr subtask ){
	subtask->addPredecessor( subtasks_.back() );
	subtasks_.back()->addSuccessor( subtask );
	subtasks_.push_back( subtask );
}

bool moveit::task_constructor::Task::plan(){
	return false;
}
