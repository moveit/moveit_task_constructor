#include <moveit_task_constructor/subtasks/gripper.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/move_group_interface/move_group_interface.h>

moveit::task_constructor::subtasks::Gripper::Gripper(std::string name)
: moveit::task_constructor::SubTask::SubTask(name)
{}

void
moveit::task_constructor::subtasks::Gripper::setGroup(std::string group){
	group_= group;
	mgi_.reset(new moveit::planning_interface::MoveGroupInterface(group));
}

void
moveit::task_constructor::subtasks::Gripper::setFrom(std::string named_target){
	from_named_target_= named_target;
}

void
moveit::task_constructor::subtasks::Gripper::setTo(std::string named_target){
	to_named_target_= named_target;
}

bool
moveit::task_constructor::subtasks::Gripper::canCompute(){
	return hasBeginning();
}

bool
moveit::task_constructor::subtasks::Gripper::compute(){
	InterfaceState& start= fetchStateBeginning();

	moveit_msgs::RobotState state;
	moveit::core::robotStateToRobotStateMsg(start.state->getCurrentState(), state);
	mgi_->setStartState(state);

	mgi_->setNamedTarget(to_named_target_);

	moveit::planning_interface::MoveGroupInterface::Plan plan;

	if(!mgi_->plan(plan))
		return false;

	// construct trajectory
	auto traj= std::make_shared<robot_trajectory::RobotTrajectory>(scene_->getRobotModel(), group_);
	traj->setRobotTrajectoryMsg(start.state->getCurrentState(), plan.trajectory_);

   // construct end state
	robot_state::RobotState end_state(start.state->getCurrentState());
	end_state.setVariablePositions(
		plan.trajectory_.joint_trajectory.joint_names,
		plan.trajectory_.joint_trajectory.points.back().positions);
	planning_scene::PlanningScenePtr end_scene(start.state->diff());
	end_scene->setCurrentState(end_state);

   // finish subtask
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(traj);
	trajectory.connectToBeginning(start);
	sendForward(trajectory, end_scene);

	return true;
}
