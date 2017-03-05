#include <moveit_task_constructor/subtasks/gripper.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

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

void
moveit::task_constructor::subtasks::Gripper::graspObject(std::string object){
	object_= object;
}

bool
moveit::task_constructor::subtasks::Gripper::canCompute(){
	return hasBeginning();
}

bool
moveit::task_constructor::subtasks::Gripper::compute(){
	InterfaceState& start= fetchStateBeginning();

	planning_scene::PlanningScenePtr scene(start.state->diff());

	mgi_->setNamedTarget(to_named_target_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	if( !object_.empty() ){
		scene->getAllowedCollisionMatrixNonConst().setEntry(object_, mgi_->getLinkNames(), true);
	}

	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(scene, req, res))
		return false;

	// set end state
	robot_state::RobotState end_state(res.trajectory_->getLastWayPoint());
	scene->setCurrentState(end_state);

	// finish subtask
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(res.trajectory_);
	trajectory.connectToBeginning(start);
	sendForward(trajectory, scene);

	return true;
}
