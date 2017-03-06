#include <moveit_task_constructor/subtasks/move.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

moveit::task_constructor::subtasks::Move::Move(std::string name)
: moveit::task_constructor::SubTask::SubTask(name),
  timeout_(5.0)
{}

void
moveit::task_constructor::subtasks::Move::setGroup(std::string group){
	group_= group;
	mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_);
}

void
moveit::task_constructor::subtasks::Move::setLink(std::string link){
	link_= link;
}

void
moveit::task_constructor::subtasks::Move::setPlannerId(std::string planner){
	planner_id_= planner;
}

void
moveit::task_constructor::subtasks::Move::setTimeout(double timeout){
	timeout_= timeout;
}

/* TODO: implement this in compute
void
moveit::task_constructor::subtasks::Move::setFrom(std::string named_target){
	from_named_target_= named_target;
}

void
moveit::task_constructor::subtasks::Move::setTo(std::string named_target){
	to_named_target_= named_target;
}
*/

bool
moveit::task_constructor::subtasks::Move::canCompute(){
	return hasStatePair();
}

bool
moveit::task_constructor::subtasks::Move::compute(){
	assert( scene_->getRobotModel() );

	std::pair<InterfaceState&, InterfaceState&> state_pair= fetchStatePair();

	mgi_->setJointValueTarget(state_pair.second.state->getCurrentState());
	if( !planner_id_.empty() )
		mgi_->setPlannerId(planner_id_);
	mgi_->setPlanningTime(timeout_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	ros::Duration(4.0).sleep();
	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(state_pair.first.state, req, res))
		return false;

	// finish subtask
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(res.trajectory_);
	trajectory.hasBeginning(state_pair.first);
	trajectory.hasEnding(state_pair.second);

	return true;
}
