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
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("object");
	p.declare<std::string>("eef_grasp_pose");
	p.declare<double>("timeout", 0.1);
	p.declare<uint32_t>("max_ik_solutions", 1);
	p.declare<geometry_msgs::TransformStamped>("grasp_frame", geometry_msgs::TransformStamped(), "robot frame to use for grasping");
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
	p.declare<bool>("ignore_collisions", false);
}

void GenerateGraspPose::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Generator::init(scene);
	scene_ = scene;
}

void GenerateGraspPose::setGroup(std::string group){
	setProperty("group", group);
}

void GenerateGraspPose::setEndEffector(std::string eef){
	setProperty("eef", eef);
}

void GenerateGraspPose::setGripperGraspPose(std::string pose_name){
	setProperty("eef_grasp_pose", pose_name);
}

void GenerateGraspPose::setObject(std::string object){
	setProperty("object", object);
}

void GenerateGraspPose::setGraspFrame(const geometry_msgs::TransformStamped &frame){
	setProperty("grasp_frame", frame);
}
void GenerateGraspPose::setGraspFrame(const Eigen::Affine3d &transform, const std::string &link)
{
	geometry_msgs::TransformStamped frame;
	frame.header.frame_id = link;
	tf::transformEigenToMsg(transform, frame.transform);
	setGraspFrame(frame);
}

void GenerateGraspPose::setTimeout(double timeout){
	setProperty("timeout", timeout);
}

void GenerateGraspPose::setAngleDelta(double delta){
	setProperty("angle_delta", delta);
}

void GenerateGraspPose::setMaxIKSolutions(uint32_t n){
	setProperty("max_ik_solutions", n);
}

void GenerateGraspPose::setIgnoreCollisions(bool flag)
{
	setProperty("ignore_collisions", flag);
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
	const auto& props = properties();
	double remaining_time = props.get<double>("timeout");

	const std::string& eef = props.get<std::string>("eef");
	const std::string& group = props.get<std::string>("group");

	assert(scene_->getRobotModel()->hasEndEffector(eef) && "The specified end effector is not defined in the srdf");

	planning_scene::PlanningScenePtr grasp_scene = scene_->diff();
	robot_state::RobotState &grasp_state = grasp_scene->getCurrentStateNonConst();

	const moveit::core::JointModelGroup* jmg_eef= grasp_state.getRobotModel()->getEndEffector(eef);

	const moveit::core::JointModelGroup* jmg_active= group.empty()
		? grasp_state.getJointModelGroup(jmg_eef->getEndEffectorParentGroup().first)
		: grasp_state.getJointModelGroup(group);

	geometry_msgs::TransformStamped grasp_frame = props.get<geometry_msgs::TransformStamped>("grasp_frame");
	const std::string &link_name = jmg_eef->getEndEffectorParentGroup().second;
	if (grasp_frame.header.frame_id.empty())
		grasp_frame.header.frame_id = link_name;
	Eigen::Affine3d grasp_pose;
	tf::transformMsgToEigen(grasp_frame.transform, grasp_pose);

	if (grasp_frame.header.frame_id != link_name) {
		// convert grasp_pose to transform relative to link (instead of frame_id)
		const Eigen::Affine3d link_pose = scene_->getFrameTransform(link_name);
		if(link_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			throw std::runtime_error("requested link does not exist or could not be retrieved");
		const Eigen::Affine3d frame_pose = scene_->getFrameTransform(grasp_frame.header.frame_id);
		if(frame_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			throw std::runtime_error("requested frame does not exist or could not be retrieved");
		grasp_pose = link_pose.inverse() * frame_pose * grasp_pose;
	}
	grasp_pose = grasp_pose.inverse(); // invert once

	const std::string& eef_grasp_pose = props.get<std::string>("eef_grasp_pose");
	if(!eef_grasp_pose.empty()){
		grasp_state.setToDefaultValues(jmg_eef, eef_grasp_pose);
	}

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			scene_,
			props.get<bool>("ignore_collisions"),
			&previous_solutions_,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);

	const Eigen::Affine3d object_pose = scene_->getFrameTransform(props.get<std::string>("object"));
	if(object_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	uint32_t max_ik_solutions = props.get<uint32_t>("max_ik_solutions");
	while( canCompute() ){
		if( remaining_time <= 0.0 || (max_ik_solutions != 0 && previous_solutions_.size() >= max_ik_solutions)){
			std::cout << "computed angle " << current_angle_
			          << " with " << previous_solutions_.size()
			          << " cached ik solutions"
			          << " and " << remaining_time << "s left" << std::endl;
			current_angle_+= props.get<double>("angle_delta");
			remaining_time = props.get<double>("timeout");
			tried_current_state_as_seed_= false;
			previous_solutions_.clear();
			continue;
		}

		// rotate object pose about z-axis
		Eigen::Affine3d goal_pose = object_pose * Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) * grasp_pose;

		if(tried_current_state_as_seed_)
			grasp_state.setToRandomPositions(jmg_active);
		tried_current_state_as_seed_= true;

		auto now= std::chrono::steady_clock::now();
		bool succeeded= grasp_state.setFromIK(jmg_active, goal_pose, link_name, 1, remaining_time, is_valid);
		remaining_time-= std::chrono::duration<double>(std::chrono::steady_clock::now()- now).count();

		if(succeeded) {
			previous_solutions_.emplace_back();
			grasp_state.copyJointGroupPositions(jmg_active, previous_solutions_.back());
			spawn(InterfaceState(grasp_scene), 0.0);
			return true;
		}
	}

	return false;
}

} } }
