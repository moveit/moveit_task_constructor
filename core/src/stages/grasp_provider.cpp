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

/* Authors: Michael Goerner, Artur Karoly */

#include <moveit/task_constructor/stages/grasp_provider.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_msgs/Grasp.h>

namespace moveit {
namespace task_constructor {
namespace stages {

//  -------------------
//  GraspProviderDefault
//  -------------------

GraspProviderDefault::GraspProviderDefault(const std::string& name) : GraspProviderBase(name){
	auto& p = properties();
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
}

void GraspProviderDefault::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GraspProviderBase::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check angle_delta
	if (props.get<double>("angle_delta") == 0.)
		errors.push_back(*this, "angle_delta must be non-zero");

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef))
		errors.push_back(*this, "unknown end effector: " + eef);
	else {
		// TODO: Validate eef_poses specified in grasps
		if (props.get<std::vector<moveit_msgs::Grasp>>("grasps").empty())
			errors.push_back(*this, "no grasp provided");
	}

	if (errors)
		throw errors;
}

void GraspProviderDefault::compute() {
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	// set end effector pose
	const auto& props = properties();
	const std::vector<moveit_msgs::Grasp>& grasps_ = props.get<std::vector<moveit_msgs::Grasp>>("grasps");
	const std::string& eef = props.get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

	geometry_msgs::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = props.get<std::string>("object");

	double current_angle = 0.0;
	while (current_angle < 2. * M_PI && current_angle > -2. * M_PI) {
		// rotate object pose about z-axis
		Eigen::Isometry3d target_pose(Eigen::AngleAxisd(current_angle, Eigen::Vector3d::UnitZ()));
		current_angle += props.get<double>("angle_delta");

		InterfaceState state(scene);
		tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
		state.properties().set("target_pose", target_pose_msg);
		state.properties().set("approach_direction", grasps_[0].pre_grasp_approach.direction);
		state.properties().set("approach_min_dist", static_cast<double>(grasps_[0].pre_grasp_approach.min_distance));
		state.properties().set("approach_max_dist", static_cast<double>(grasps_[0].pre_grasp_approach.desired_distance));

		state.properties().set("retreat_direction", grasps_[0].post_grasp_retreat.direction);
		state.properties().set("retreat_min_dist", static_cast<double>(grasps_[0].post_grasp_retreat.min_distance));
		state.properties().set("retreat_max_dist", static_cast<double>(grasps_[0].post_grasp_retreat.desired_distance));

		std::map<std::string, double> hand_open_pose;
		std::map<std::string, double> hand_close_pose;

		std::vector<std::string> hand_joint_names = grasps_[0].pre_grasp_posture.joint_names;
		std::vector<double> open_pose = grasps_[0].pre_grasp_posture.points[0].positions;
		std::vector<double> close_pose = grasps_[0].grasp_posture.points[0].positions;

		std::transform(hand_joint_names.begin(), hand_joint_names.end(), open_pose.begin(), std::inserter(hand_open_pose, hand_open_pose.end()), std::make_pair<std::string const&, double const&>);
		std::transform(hand_joint_names.begin(), hand_joint_names.end(), close_pose.begin(), std::inserter(hand_close_pose, hand_close_pose.end()), std::make_pair<std::string const&, double const&>);

		// TODO: Raise exception if the sizes do not match

		state.properties().set("hand_open_pose", hand_open_pose);
		state.properties().set("hand_close_pose", hand_close_pose);

		// props.exposeTo(state.properties(), { "pregrasp", "grasp" });

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

		spawn(std::move(state), std::move(trajectory));
	}
}


//  -------------------
//  GraspProviderFixedPoses
//  -------------------


GraspProviderFixedPoses::GraspProviderFixedPoses(const std::string& name) : GraspProviderBase(name){}

void GraspProviderFixedPoses::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GraspProviderBase::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef))
		errors.push_back(*this, "unknown end effector: " + eef);
	else {
		// TODO: Validate eef_poses specified in grasps
		if (props.get<std::vector<moveit_msgs::Grasp>>("grasps").empty())
			errors.push_back(*this, "no grasp provided");
	}

	if (errors)
		throw errors;
}

void GraspProviderFixedPoses::compute() {
	if (upstream_solutions_.empty())
		return;

	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	const std::vector<moveit_msgs::Grasp>& grasps_ = properties().get<std::vector<moveit_msgs::Grasp>>("grasps");

	for (moveit_msgs::Grasp grasp_ : grasps_){

		geometry_msgs::PoseStamped target_pose  = grasp_.grasp_pose;
		if (target_pose.header.frame_id.empty())
			target_pose.header.frame_id = scene->getPlanningFrame();
		else if (!scene->knowsFrameTransform(target_pose.header.frame_id)) {
			ROS_WARN_NAMED("GeneratePose", "Unknown frame: '%s'", target_pose.header.frame_id.c_str());
			return;
		}

		InterfaceState state(scene);
		state.properties().set("target_pose", target_pose);
		state.properties().set("approach_direction", grasp_.pre_grasp_approach.direction);
		state.properties().set("approach_min_dist", static_cast<double>(grasp_.pre_grasp_approach.min_distance));
		state.properties().set("approach_max_dist", static_cast<double>(grasp_.pre_grasp_approach.desired_distance));

		state.properties().set("retreat_direction", grasp_.post_grasp_retreat.direction);
		state.properties().set("retreat_min_dist", static_cast<double>(grasp_.post_grasp_retreat.min_distance));
		state.properties().set("retreat_max_dist", static_cast<double>(grasp_.post_grasp_retreat.desired_distance));

		std::map<std::string, double> hand_open_pose;
		std::map<std::string, double> hand_close_pose;

		std::vector<std::string> hand_joint_names = grasp_.pre_grasp_posture.joint_names;
		std::vector<double> open_pose = grasp_.pre_grasp_posture.points[0].positions;
		std::vector<double> close_pose = grasp_.grasp_posture.points[0].positions;

		std::transform(hand_joint_names.begin(), hand_joint_names.end(), open_pose.begin(), std::inserter(hand_open_pose, hand_open_pose.end()), std::make_pair<std::string const&, double const&>);
		std::transform(hand_joint_names.begin(), hand_joint_names.end(), close_pose.begin(), std::inserter(hand_close_pose, hand_close_pose.end()), std::make_pair<std::string const&, double const&>);

		// TODO: Raise exception if the sizes do not match

		state.properties().set("hand_open_pose", hand_open_pose);
		state.properties().set("hand_close_pose", hand_close_pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);

		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "grasp frame");

		spawn(std::move(state), std::move(trajectory));
	}
}



}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit

/// register plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit::task_constructor::stages::GraspProviderDefault, moveit::task_constructor::stages::GraspProviderBase)
PLUGINLIB_EXPORT_CLASS(moveit::task_constructor::stages::GraspProviderFixedPoses, moveit::task_constructor::stages::GraspProviderBase)
