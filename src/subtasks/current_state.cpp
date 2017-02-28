#include <moveit_task_constructor/subtasks/current_state.h>

bool
moveit::task_constructor::subtasks::CurrentState::canCompute(){
	return true;
}

bool
moveit::task_constructor::subtasks::CurrentState::compute(){
	return false;
}
