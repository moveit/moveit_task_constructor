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

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/storage.h>

#include <ros/ros.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

#include <chrono>
#include <functional>

namespace moveit { namespace task_constructor { namespace stages {

GenerateGraspPose::GenerateGraspPose(std::string name)
: Generator(name)
{
}

void GenerateGraspPose::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Generator::init(scene);
	scene_ = scene;
}

void GenerateGraspPose::setGroup(std::string group){
	group_= group;
}

void GenerateGraspPose::setLink(std::string ik_link){
	ik_link_= ik_link;
}

void GenerateGraspPose::setEndEffector(std::string eef){
	eef_= eef;
}

void GenerateGraspPose::setGripperGraspPose(std::string pose_name){
	gripper_grasp_pose_= pose_name;
}

void GenerateGraspPose::setObject(std::string object){
	object_= object;
}

void GenerateGraspPose::setGraspOffset(double offset){
	grasp_offset_= offset;
}

void GenerateGraspPose::setTimeout(double timeout){
	timeout_= timeout;
	remaining_time_= timeout;
}

void GenerateGraspPose::setAngleDelta(double delta){
	angle_delta_= delta;
}

void GenerateGraspPose::setMaxIKSolutions(uint32_t n){
	max_ik_solutions_= n;
}

bool GenerateGraspPose::canCompute() const{
	return current_angle_ < 2*M_PI && current_angle_ > -2*M_PI;
}

namespace {
	bool isValid(planning_scene::PlanningSceneConstPtr scene,
	             bool ignore_collisions,
	             std::vector< std::vector<double> >* old_solutions,
	             robot_state::RobotState* state,
	             const robot_model::JointModelGroup* jmg,
	             const double* joint_positions){
		for( std::vector<double> sol : *old_solutions ){
			if( jmg->distance(joint_positions, sol.data()) < 0.1 ){
				return false;
			}
		}

		if(ignore_collisions)
			return true;

		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		if( scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName()) ){
			old_solutions->emplace_back();
			state->copyJointGroupPositions(jmg, old_solutions->back());
			return false;
		}
		return true;
	}
}

bool GenerateGraspPose::compute(){
	assert(scene_->getRobotModel()->hasEndEffector(eef_) && "The specified end effector is not defined in the srdf");

	planning_scene::PlanningScenePtr grasp_scene = scene_->diff();
	robot_state::RobotState &grasp_state = grasp_scene->getCurrentStateNonConst();

	const moveit::core::JointModelGroup* jmg_eef= grasp_state.getRobotModel()->getEndEffector(eef_);

	const moveit::core::JointModelGroup* jmg_active= group_.empty()
		? grasp_state.getJointModelGroup(jmg_eef->getEndEffectorParentGroup().first)
		: grasp_state.getJointModelGroup(group_);

	const std::string ik_link= ik_link_.empty() ? jmg_eef->getEndEffectorParentGroup().second : ik_link_;

	if(!gripper_grasp_pose_.empty()){
		grasp_state.setToDefaultValues(jmg_eef, gripper_grasp_pose_);
	}

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			scene_,
			ignore_collisions_,
			&previous_solutions_,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);

	geometry_msgs::Pose object_pose, grasp_pose;
	const Eigen::Affine3d object_pose_eigen= scene_->getFrameTransform(object_);
	if(object_pose_eigen.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	tf::poseEigenToMsg(object_pose_eigen, object_pose);

	while( canCompute() ){
		if( remaining_time_ <= 0.0 || (max_ik_solutions_ != 0 && previous_solutions_.size() >= max_ik_solutions_)){
			std::cout << "computed angle " << current_angle_
			          << " with " << previous_solutions_.size()
			          << " cached ik solutions"
			          << " and " << remaining_time_ << "s left" << std::endl;
			current_angle_+= angle_delta_;
			remaining_time_= timeout_;
			tried_current_state_as_seed_= false;
			previous_solutions_.clear();
			continue;
		}

		grasp_pose= object_pose;

		grasp_pose.position.x-= grasp_offset_*cos(current_angle_);
		grasp_pose.position.y-= grasp_offset_*sin(current_angle_);
		grasp_pose.orientation= tf::createQuaternionMsgFromRollPitchYaw(M_PI, 0.0, current_angle_);

		if(tried_current_state_as_seed_)
			grasp_state.setToRandomPositions(jmg_active);
		tried_current_state_as_seed_= true;

		auto now= std::chrono::steady_clock::now();
		bool succeeded= grasp_state.setFromIK(jmg_active, grasp_pose, ik_link, 1, remaining_time_, is_valid);
		remaining_time_-= std::chrono::duration<double>(std::chrono::steady_clock::now()- now).count();

		if(succeeded) {
			previous_solutions_.emplace_back();
			grasp_state.copyJointGroupPositions(jmg_active, previous_solutions_.back());
			spawn(InterfaceState(grasp_scene));
			return true;
		}
	}

	return false;
}

} } }
