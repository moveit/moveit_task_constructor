#include <moveit_task_constructor/subtasks/current_state.h>

namespace moveit { namespace task_constructor { namespace subtasks {

CurrentState::CurrentState(std::string name)
: SubTask(name)
{
	ran_= false;
}

bool CurrentState::canCompute(){
	return !ran_;
}

bool CurrentState::compute(){
	ran_= true;

	assert( scene_ );

	// empty trajectory ref -> this node only produces states
	robot_trajectory::RobotTrajectoryPtr traj;
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(traj);

	sendBothWays(trajectory, scene_);

	return true;
}

} } }
