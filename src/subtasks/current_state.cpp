#include <moveit_task_constructor/subtasks/current_state.h>

moveit::task_constructor::subtasks::CurrentState::CurrentState(std::string name)
: moveit::task_constructor::SubTask::SubTask(name)
{
	ran_= false;
}

bool
moveit::task_constructor::subtasks::CurrentState::canCompute(){
	return !ran_;
}

bool
moveit::task_constructor::subtasks::CurrentState::compute(){
	ran_= true;

	assert( scene_ );

	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr traj;
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(traj);

	sendBothWays(trajectory, scene_);

	return true;
}
