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
moveit::task_constructor::subtasks::Gripper::setEndEffector(std::string eef){
	eef_= eef;
}

void
moveit::task_constructor::subtasks::Gripper::setAttachLink(std::string link){
	attach_link_= link;
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
moveit::task_constructor::subtasks::Gripper::graspObject(std::string grasp_object){
	grasp_object_= grasp_object;
}

bool
moveit::task_constructor::subtasks::Gripper::canCompute(){
	return hasBeginning();
}

bool
moveit::task_constructor::subtasks::Gripper::compute(){
	assert( scene_->getRobotModel() );

	if(!mgi_){
		assert( scene_->getRobotModel()->hasEndEffector(eef_) && "no end effector with that name defined in srdf" );
		const moveit::core::JointModelGroup* jmg= scene_->getRobotModel()->getEndEffector(eef_);
		const std::string group_name= jmg->getName();
		mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);

		if( attach_link_.empty() ){
			attach_link_= jmg->getEndEffectorParentGroup().second;
		}
	}

	InterfaceState& start= fetchStateBeginning();

	planning_scene::PlanningScenePtr scene(start.state->diff());

	mgi_->setNamedTarget(to_named_target_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	if( !grasp_object_.empty() ){
		scene->getAllowedCollisionMatrixNonConst().setEntry(grasp_object_, mgi_->getLinkNames(), true);
	}

	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(scene, req, res))
		return false;

	// set end state
	robot_state::RobotState end_state(res.trajectory_->getLastWayPoint());
	scene->setCurrentState(end_state);

	// attach object
	if( !grasp_object_.empty() ){
		moveit_msgs::AttachedCollisionObject obj;
		obj.link_name= attach_link_;
		obj.object.id= grasp_object_;
		scene->processAttachedCollisionObjectMsg(obj);
	}

	// finish subtask
	moveit::task_constructor::SubTrajectory& trajectory= addTrajectory(res.trajectory_);
	trajectory.hasBeginning(start);
	sendForward(trajectory, scene);

	return true;
}
