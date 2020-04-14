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

namespace moveit {
namespace task_constructor {
namespace stages {

GenerateGraspPose::GenerateGraspPose(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("object");
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");

	p.declare<boost::any>("pregrasp", "pregrasp posture");
	p.declare<boost::any>("grasp", "grasp posture");
}

void GenerateGraspPose::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
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
	else if (props.property("pregrasp").defined()) {
		// if pregrasp pose is defined, check if it's valid
		const moveit::core::JointModelGroup* eef_jmg = robot_model->getEndEffector(eef);
		const boost::any& pregrasp_prop = props.get("pregrasp");
		if (pregrasp_prop.type() == typeid(std::string)) {
			// check if the specified pregrasp pose is a valid named target
			const auto& pregrasp_name = boost::any_cast<std::string>(pregrasp_prop);
			std::map<std::string, double> m;
			if (!eef_jmg->getVariableDefaultPositions(pregrasp_name, m))
				errors.push_back(*this, "pregrasp is set to unknown end effector pose: " + pregrasp_name);
		} else if (pregrasp_prop.type() == typeid(sensor_msgs::JointState)) {
			// check if the specified pregrasp pose is a valid named target
			const auto& pregrasp_state = boost::any_cast<sensor_msgs::JointState>(pregrasp_prop);
			if (pregrasp_state.name.size() == pregrasp_state.position.size() &&
			    pregrasp_state.name.size() == pregrasp_state.velocity.size() &&
			    pregrasp_state.name.size() == pregrasp_state.effort.size()) {
				// We only apply the joint state for for joints of the end effector
				sensor_msgs::JointState eef_state;
				eef_state.header = pregrasp_state.header;
				for (size_t i = 0; i < pregrasp_state.name.size(); ++i) {
					if (eef_jmg->hasJointModel(pregrasp_state.name[i])) {
						eef_state.name.push_back(pregrasp_state.name[i]);
						eef_state.position.push_back(pregrasp_state.position[i]);
						eef_state.velocity.push_back(pregrasp_state.velocity[i]);
						eef_state.effort.push_back(pregrasp_state.effort[i]);
					}
				}
				if (eef_state.name.empty())
					errors.push_back(*this, "pregrasp JointState doesn't contain joint values for end effector group");
				else
					properties().set("pregrasp_state", eef_state);  // Override property with filtered joint state
			} else {
				errors.push_back(*this, "pregrasp JointState is malformed - size mismatch between joint names and values");
			}
		}
	}

	if (errors)
		throw errors;
}

void GenerateGraspPose::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	if (!scene->knowsFrameTransform(object)) {
		const std::string msg = "object '" + object + "' not in scene";
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else
			ROS_WARN_STREAM_NAMED("GenerateGraspPose", msg);
		return;
	}

	upstream_solutions_.push(&s);
}

void GenerateGraspPose::compute() {
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	// set end effector pose
	const auto& props = properties();
	const std::string& eef = props.get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

	// Apply pregrasp target or joint state if defined
	const boost::any& pregrasp_prop = props.get("pregrasp");
	if (!pregrasp_prop.empty()) {
		robot_state::RobotState& current_state = scene->getCurrentStateNonConst();
		if (pregrasp_prop.type() == typeid(std::string)) {
			current_state.setToDefaultValues(jmg, boost::any_cast<std::string>(pregrasp_prop));
		} else if (pregrasp_prop.type() == typeid(sensor_msgs::JointState)) {
			const auto& pregrasp_state = boost::any_cast<sensor_msgs::JointState>(pregrasp_prop);
			current_state.setVariablePositions(pregrasp_state.name, pregrasp_state.position);
		}
	}

	geometry_msgs::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = props.get<std::string>("object");

	double current_angle_ = 0.0;
	while (current_angle_ < 2. * M_PI && current_angle_ > -2. * M_PI) {
		// rotate object pose about z-axis
		Eigen::Isometry3d target_pose(Eigen::AngleAxisd(current_angle_, Eigen::Vector3d::UnitZ()));
		current_angle_ += props.get<double>("angle_delta");

		InterfaceState state(scene);
		tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
		state.properties().set("target_pose", target_pose_msg);
		props.exposeTo(state.properties(), { "pregrasp", "grasp" });

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle_));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

		spawn(std::move(state), std::move(trajectory));
	}
}
}
}
}
