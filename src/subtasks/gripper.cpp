#include <moveit_task_constructor/subtasks/gripper.h>
#include <moveit_task_constructor/storage.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace subtasks {

Gripper::Gripper(std::string name)
   : PropagatingForward(name)
{}

void Gripper::setEndEffector(std::string eef){
	eef_= eef;
}

void Gripper::setAttachLink(std::string link){
	attach_link_= link;
}

void Gripper::setFrom(std::string named_target){
	// TODO: this direction isn't handled yet
	from_named_target_= named_target;
}

void Gripper::setTo(std::string named_target){
	to_named_target_= named_target;
}

void Gripper::graspObject(std::string grasp_object){
	grasp_object_= grasp_object;
}

bool Gripper::canCompute() const{
	return hasBeginning();
}

bool Gripper::compute(){
	assert(scene()->getRobotModel());

	if(!mgi_){
		assert(scene()->getRobotModel()->hasEndEffector(eef_) && "no end effector with that name defined in srdf");
		const moveit::core::JointModelGroup* jmg= scene()->getRobotModel()->getEndEffector(eef_);
		const std::string group_name= jmg->getName();
		mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);

		if( attach_link_.empty() ){
			attach_link_= jmg->getEndEffectorParentGroup().second;
		}
	}

	const InterfaceState& start= fetchStateBeginning();

	planning_scene::PlanningScenePtr scene(start.state->diff());

	mgi_->setNamedTarget(to_named_target_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	if( !grasp_object_.empty() ){
		scene->getAllowedCollisionMatrixNonConst().setEntry(grasp_object_, mgi_->getLinkNames(), true);
	}

	::planning_interface::MotionPlanResponse res;
	if(!planner()->generatePlan(scene, req, res))
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
	sendForward(res.trajectory_, start, scene);

	return true;
}

} } }
