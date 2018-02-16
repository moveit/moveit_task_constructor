/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit { namespace task_constructor { namespace stages {

GenerateGraspPose::GenerateGraspPose(std::string name)
: Generator(name)
{
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("pregrasp", "name of end-effector's pregrasp pose");
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

void GenerateGraspPose::setNamedPose(const std::string &pose_name){
	setProperty("pregrasp", pose_name);
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
	const std::string& joint_pose = props.get<std::string>("pregrasp");
	if(!joint_pose.empty()){
		robot_state.setToDefaultValues(jmg , joint_pose);
	}

	geometry_msgs::TransformStamped tool2grasp_msg = props.get<geometry_msgs::TransformStamped>("tool_to_grasp_tf");
	const std::string &link_name = jmg->getEndEffectorParentGroup().second;
	if (tool2grasp_msg.header.frame_id.empty()) // if no frame_id is given
		tool2grasp_msg.header.frame_id = link_name; // interpret the transform w.r.t. eef link frame
	Eigen::Affine3d to_grasp;
	Eigen::Affine3d grasp2tool, grasp2link;
	tf::transformMsgToEigen(tool2grasp_msg.transform, to_grasp);
	grasp2tool = to_grasp.inverse();

	const robot_model::LinkModel* link = robot_state.getLinkModel(link_name);
	if (!link) throw std::runtime_error("requested link '" + link_name + "' does not exist");

	if (tool2grasp_msg.header.frame_id != link_name) {
		// convert to_grasp into transform w.r.t. link (instead of tool frame_id)
		const robot_model::LinkModel* tool_link = robot_state.getLinkModel(tool2grasp_msg.header.frame_id);
		if (!tool_link) throw std::runtime_error("requested frame '" + tool2grasp_msg.header.frame_id + "' is not a robot link");

		const Eigen::Affine3d link_pose = robot_state.getGlobalLinkTransform(link);
		const Eigen::Affine3d tool_pose = robot_state.getGlobalLinkTransform(tool_link);
		to_grasp = link_pose.inverse() * tool_pose * to_grasp;
		grasp2link = to_grasp.inverse();
	} else
		grasp2link = grasp2tool;

	const std::string& object_name = props.get<std::string>("object");
	if (!scene_->knowsFrameTransform(object_name))
		throw std::runtime_error("requested object does not exist or could not be retrieved");
	const Eigen::Affine3d object_pose = scene_->getFrameTransform(object_name);

	while( canCompute() ) {
		// rotate object pose about z-axis
		Eigen::Affine3d grasp_pose = object_pose * Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ());
		Eigen::Affine3d link_pose = grasp_pose * grasp2link;
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene_);
		geometry_msgs::PoseStamped goal_pose_msg;
		goal_pose_msg.header.frame_id = link_name;
		tf::poseEigenToMsg(link_pose, goal_pose_msg.pose);
		state.properties().set("target_pose", goal_pose_msg);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setName(std::to_string(current_angle_));

		// add frame at grasp pose
		goal_pose_msg.header.frame_id = scene_->getPlanningFrame();
		tf::poseEigenToMsg(grasp_pose, goal_pose_msg.pose);
		rviz_marker_tools::appendFrame(trajectory.markers(), goal_pose_msg, 0.1, "grasp frame");

		// add end-effector marker visualizing the pose of the end-effector, including all rigidly connected parent links
		const robot_model::LinkModel* link = robot_state.getLinkModel(link_name);
		const robot_model::LinkModel* parent = robot_model::RobotModel::getRigidlyConnectedParentLinkModel(link);
		if (parent != link)  // transform pose into pose suitable to place parent
			link_pose = link_pose * robot_state.getGlobalLinkTransform(link).inverse() * robot_state.getGlobalLinkTransform(parent);

		robot_state.updateStateWithLinkAt(parent, link_pose);
		auto appender = [&trajectory](visualization_msgs::Marker& marker, const std::string& name) {
			marker.ns = "grasp eef";
			marker.color.a *= 0.5;
			trajectory.markers().push_back(marker);
		};
		generateVisualMarkers(robot_state, appender, parent->getParentJointModel()->getDescendantLinkModels());

		spawn(std::move(state), std::move(trajectory));
		return true;
	}

	return false;
}

} } }
