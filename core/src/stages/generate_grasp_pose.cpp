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

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

GenerateGraspPose::GenerateGraspPose(std::string name)
: Generator(name)
{
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector group");
	p.declare<std::string>("eef_named_pose");
	p.declare<std::string>("object");
	p.declare<geometry_msgs::TransformStamped>("tool_to_grasp_tf", geometry_msgs::TransformStamped(), "transform from robot tool frame to grasp frame");
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
}

void GenerateGraspPose::init(const planning_scene::PlanningSceneConstPtr &scene)
{
	Generator::init(scene);
	scene_ = scene->diff();
}

void GenerateGraspPose::setEndEffector(const std::string &eef){
	setProperty("eef", eef);
}

void GenerateGraspPose::setGripperGraspPose(const std::string &pose_name){
	setProperty("eef_named_pose", pose_name);
}

void GenerateGraspPose::setObject(const std::string &object){
	setProperty("object", object);
}

void GenerateGraspPose::setToolToGraspTF(const geometry_msgs::TransformStamped &transform){
	setProperty("tool_to_grasp_tf", transform);
}

void GenerateGraspPose::setToolToGraspTF(const Eigen::Affine3d &transform, const std::string &link){
	geometry_msgs::TransformStamped stamped;
	stamped.header.frame_id = link;
	stamped.child_frame_id = "grasp_frame";
	tf::transformEigenToMsg(transform, stamped.transform);
	setToolToGraspTF(stamped);
}

void GenerateGraspPose::setAngleDelta(double delta){
	setProperty("angle_delta", delta);
}

bool GenerateGraspPose::canCompute() const{
	return current_angle_ < 2*M_PI && current_angle_ > -2*M_PI;
}

bool GenerateGraspPose::compute(){
	const auto& props = properties();
	const std::string& eef = props.get<std::string>("eef");

	assert(scene_->getRobotModel()->hasEndEffector(eef) && "The specified end effector is not defined in the srdf");
	const moveit::core::JointModelGroup* jmg = scene_->getRobotModel()->getEndEffector(eef);

	robot_state::RobotState &robot_state = scene_->getCurrentStateNonConst();
	const std::string& eef_named_pose = props.get<std::string>("eef_named_pose");
	if(!eef_named_pose.empty()){
		robot_state.setToDefaultValues(jmg , eef_named_pose);
	}

	geometry_msgs::TransformStamped grasp_frame = props.get<geometry_msgs::TransformStamped>("tool_to_grasp_tf");
	const std::string &link_name = jmg ->getEndEffectorParentGroup().second;
	if (grasp_frame.header.frame_id.empty()) // if no frame_id is given
		grasp_frame.header.frame_id = link_name; // interpret the transform w.r.t. eef link frame
	Eigen::Affine3d grasp_pose;
	tf::transformMsgToEigen(grasp_frame.transform, grasp_pose);

	if (grasp_frame.header.frame_id != link_name) {
		// convert grasp_pose into transform w.r.t. link (instead of frame_id)
		const Eigen::Affine3d link_pose = scene_->getFrameTransform(link_name);
		if(link_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			throw std::runtime_error("requested link does not exist or could not be retrieved");
		const Eigen::Affine3d frame_pose = scene_->getFrameTransform(grasp_frame.header.frame_id);
		if(frame_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
			throw std::runtime_error("requested frame does not exist or could not be retrieved");
		grasp_pose = link_pose.inverse() * frame_pose * grasp_pose;
	}
	grasp_pose = grasp_pose.inverse(); // invert once

	const Eigen::Affine3d object_pose = scene_->getFrameTransform(props.get<std::string>("object"));
	if(object_pose.matrix().cwiseEqual(Eigen::Affine3d::Identity().matrix()).all())
		throw std::runtime_error("requested object does not exist or could not be retrieved");

	while( canCompute() ){
		// rotate object pose about z-axis
		Eigen::Affine3d goal_pose = object_pose * Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()) * grasp_pose;
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene_);
		geometry_msgs::PoseStamped goal_pose_msg;
		goal_pose_msg.header.frame_id = link_name;
		tf::poseEigenToMsg(goal_pose, goal_pose_msg.pose);
		state.properties().set("target_pose", goal_pose_msg);

		spawn(std::move(state));
		return true;
	}

	return false;
}

} } }
