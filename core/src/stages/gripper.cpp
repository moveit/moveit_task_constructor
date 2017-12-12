/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael Goerner */

#include <moveit/task_constructor/stages/gripper.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/task.h>

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

void Gripper::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	PropagatingEitherWay::init(scene);
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
	scene = state.scene()->diff();
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
	sendForward(from, InterfaceState(to), trajectory, cost);
	return true;
}

bool Gripper::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from;
	robot_trajectory::RobotTrajectoryPtr trajectory;
	double cost = 0;

	if (!compute(to, from, trajectory, cost, BACKWARD))
		return false;
	sendBackward(InterfaceState(from), to, trajectory, cost);
	return true;
}

} } }
