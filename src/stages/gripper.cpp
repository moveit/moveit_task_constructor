#include <moveit_task_constructor/stages/gripper.h>
#include <moveit_task_constructor/storage.h>
#include <moveit_task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>

namespace moveit { namespace task_constructor { namespace stages {

Gripper::Gripper(std::string name)
   : PropagatingEitherWay(name)
{}

bool Gripper::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	planner_ = Task::createPlanner(scene->getRobotModel());
}

void Gripper::setEndEffector(std::string eef){
	eef_= eef;
}

void Gripper::setAttachLink(std::string link){
	attach_link_= link;
}

void Gripper::setFrom(std::string named_target){
	restrictDirection(BACKWARD);
	named_target_= named_target;
}

void Gripper::setTo(std::string named_target){
	restrictDirection(FORWARD);
	named_target_= named_target;
}

void Gripper::graspObject(std::string grasp_object){
	grasp_object_= grasp_object;
}

bool Gripper::compute(const InterfaceState &state, planning_scene::PlanningScenePtr &scene,
                      robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost, Direction dir){
	scene = state.state->diff();
	assert(scene->getRobotModel());

	if(!mgi_){
		assert(scene->getRobotModel()->hasEndEffector(eef_) && "no end effector with that name defined in srdf");
		const moveit::core::JointModelGroup* jmg= scene->getRobotModel()->getEndEffector(eef_);
		const std::string group_name= jmg->getName();
		mgi_= std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name);

		if( attach_link_.empty() ){
			attach_link_= jmg->getEndEffectorParentGroup().second;
		}
	}

	mgi_->setNamedTarget(named_target_);

	::planning_interface::MotionPlanRequest req;
	mgi_->constructMotionPlanRequest(req);

	if( !grasp_object_.empty() ){
		scene->getAllowedCollisionMatrixNonConst().setEntry(grasp_object_, mgi_->getLinkNames(), true);
	}

	::planning_interface::MotionPlanResponse res;
	if(!planner_->generatePlan(scene, req, res))
		return false;

	trajectory = res.trajectory_;
	if (dir == BACKWARD) trajectory->reverse();

	scene->setCurrentState(trajectory->getLastWayPoint());

	// attach object
	if( !grasp_object_.empty() ){
		moveit_msgs::AttachedCollisionObject obj;
		obj.link_name= attach_link_;
		obj.object.id= grasp_object_;
		scene->processAttachedCollisionObjectMsg(obj);
	}

	return true;
}

bool Gripper::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr to;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost = 0;

	if (!compute(from, to, trajectory, cost, FORWARD))
		return false;
	sendForward(from, to, trajectory, cost);
	return true;
}

bool Gripper::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost = 0;

	if (!compute(to, from, trajectory, cost, BACKWARD))
		return false;
	sendBackward(from, to, trajectory, cost);
	return true;
}

} } }
