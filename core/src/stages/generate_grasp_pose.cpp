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

GenerateGraspPose::GenerateGraspPose(const std::string& name)
   : GeneratePose(name)
{
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("pregrasp", "name of end-effector's pregrasp pose");
	p.declare<std::string>("object");
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
}

void GenerateGraspPose::setEndEffector(const std::string &eef) {
	setProperty("eef", eef);
}

void GenerateGraspPose::setNamedPose(const std::string &pose_name) {
	setProperty("pregrasp", pose_name);
}

void GenerateGraspPose::setObject(const std::string &object) {
	setProperty("object", object);
}

void GenerateGraspPose::setAngleDelta(double delta){
	setProperty("angle_delta", delta);
}

void GenerateGraspPose::init(const core::RobotModelConstPtr& robot_model)
{
	InitStageException errors;
	try { GeneratePose::init(robot_model); }
	catch (InitStageException &e) { errors.append(e); }

	const auto& props = properties();

	// check angle_delta
	if (props.get<double>("angle_delta") == 0.)
		errors.push_back(*this, "angle_delta must be non-zero");

	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef))
		errors.push_back(*this, "unknown end effector: " + eef);
	else {
		// check availability of eef pose
		const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
		const std::string& name = props.get<std::string>("pregrasp");
		std::map<std::string, double> m;
		if (!jmg->getVariableDefaultPositions(name, m))
			errors.push_back(*this, "unknown end effector pose: " + name);
	}

	if (errors) throw errors;
}

void GenerateGraspPose::onNewSolution(const SolutionBase& s)
{
	planning_scene::PlanningScenePtr scene = s.end()->scene()->diff();

	// set end effector pose
	const auto& props = properties();
	const std::string& eef = props.get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

	robot_state::RobotState &robot_state = scene->getCurrentStateNonConst();
	robot_state.setToDefaultValues(jmg , props.get<std::string>("pregrasp"));

	const std::string& object_name = props.get<std::string>("object");
	if (!scene->knowsFrameTransform(object_name)) {
		ROS_WARN_STREAM_NAMED("GenerateGraspPose", "unknown object: " << object_name);
		return;
	}

	scenes_.push_back(scene);
}

bool GenerateGraspPose::compute() {
	if (scenes_.empty())
		return false;
	planning_scene::PlanningSceneConstPtr scene = scenes_[0];
	scenes_.pop_front();

	const auto& props = properties();

	geometry_msgs::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = props.get<std::string>("object");

	double current_angle_ = 0.0;
	while (current_angle_ < 2.*M_PI && current_angle_ > -2.*M_PI) {
		// rotate object pose about z-axis
		Eigen::Affine3d target_pose(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene);
		tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
		state.properties().set("target_pose", target_pose_msg);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setName(std::to_string(current_angle_));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

		spawn(std::move(state), std::move(trajectory));
	}
	return true;
}

} } }
