/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Authors: Sebastian Castro
   Desc:    generate and validate a path using the Pilz industrial motion planner
*/

#include <moveit/task_constructor/solvers/pilz.h>
#include <moveit/task_constructor/moveit_compat.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_parameterization.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/cartesian_interpolator.h> // TODO remove
#include <moveit/robot_state/conversions.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

using namespace trajectory_processing;

namespace moveit::task_constructor::solvers {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("Pilz");

Pilz::Pilz() {
	auto& p = properties();
	p.declare<std::string>("planner", "Pilz planner type (ptp, linear, circle)");
	p.declare<std::pair<std::string, geometry_msgs::msg::PoseStamped>>("arc_constraint", "arc constraint specification");
}

void Pilz::init(const core::RobotModelConstPtr& /*robot_model*/) {}

bool Pilz::plan(const planning_scene::PlanningSceneConstPtr& from, const planning_scene::PlanningSceneConstPtr& to,
                const moveit::core::JointModelGroup* jmg, double timeout, robot_trajectory::RobotTrajectoryPtr& result,
                const moveit_msgs::msg::Constraints& path_constraints) {
	const moveit::core::LinkModel* link = jmg->getOnlyOneEndEffectorTip();
	if (!link) {
		RCLCPP_WARN_STREAM(LOGGER, "no unique tip for joint model group: " << jmg->getName());
		return false;
	}

	// reach pose of forward kinematics
	return plan(from, *link, to->getCurrentState().getGlobalLinkTransform(link), jmg, timeout, result, path_constraints);
}

bool Pilz::setPlanner(const std::string& planner) {
	if (planner == "LIN" || planner == "CIRC" || planner == "PTP") {
		setProperty("planner", planner);
		return true;
	} else {
		RCLCPP_ERROR(LOGGER, "Undefined planner type %s", planner.c_str());
		return false;
	}
}

bool Pilz::setCircularArcConstraint(const std::pair<std::string, geometry_msgs::msg::PoseStamped>& constraint) {
	const std::string constraint_type = constraint.first;
	if (constraint_type == "center" || constraint_type == "interim") {
		setProperty("arc_constraint", constraint);
		return true;	
	} else {
		RCLCPP_ERROR(LOGGER, "Undefined arc constraint type %s", constraint_type.c_str());
		return false;
	}
}

bool Pilz::plan(const planning_scene::PlanningSceneConstPtr& from, const moveit::core::LinkModel& link,
                const Eigen::Isometry3d& target, const moveit::core::JointModelGroup* jmg, double timeout,
                robot_trajectory::RobotTrajectoryPtr& result, const moveit_msgs::msg::Constraints& path_constraints) {
	const auto& props = properties();
	planning_scene::PlanningScenePtr sandbox_scene = from->diff();
	const auto& robot_model = sandbox_scene->getRobotModel();

	kinematic_constraints::KinematicConstraintSet kcs(robot_model);
	kcs.add(path_constraints, sandbox_scene->getTransforms());

	auto is_valid = [&sandbox_scene, &kcs](moveit::core::RobotState* state, const moveit::core::JointModelGroup* jmg,
	                                       const double* joint_positions) {
		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		return !sandbox_scene->isStateColliding(const_cast<const moveit::core::RobotState&>(*state), jmg->getName()) &&
		       kcs.decide(*state).satisfied;
	};

	const std::string group_name = jmg->getName();
	const std::string link_name = link.getName();

	// TODO: Get from properties
	// and also set joint limits (from robot model?)
	using pilz_industrial_motion_planner::CartesianLimit;
	using pilz_industrial_motion_planner::LimitsContainer;
	auto cart_limits = CartesianLimit();
	cart_limits.setMaxRotationalVelocity(1.0);
	cart_limits.setMaxTranslationalVelocity(1.0);
	cart_limits.setMaxTranslationalAcceleration(1.0);
	cart_limits.setMaxTranslationalDeceleration(-1.0);
	auto limits = LimitsContainer();
	limits.setCartesianLimits(cart_limits);

	// Create a trajectory generator
	if (props.get("planner").empty()) {
		RCLCPP_ERROR(LOGGER, "Undefined planner.");
		return false;
	}
	const std::string& planner = props.get<std::string>("planner");
	if (planner == "PTP") {
		traj_gen_ =
		    std::make_shared<pilz_industrial_motion_planner::TrajectoryGeneratorPTP>(robot_model, limits, group_name);
	} else if (planner == "LIN") {
		traj_gen_ =
		    std::make_shared<pilz_industrial_motion_planner::TrajectoryGeneratorLIN>(robot_model, limits, group_name);
	} else if (planner == "CIRC") {
		traj_gen_ =
		    std::make_shared<pilz_industrial_motion_planner::TrajectoryGeneratorCIRC>(robot_model, limits, group_name);
	}  // else case already handled in setPlanner() and empty checking logic

	// Package up a motion plan request for Pilz
	planning_interface::MotionPlanRequest motion_request;
	motion_request.pipeline_id = "pilz_industrial_motion_planner";
	motion_request.planner_id = planner;
	motion_request.group_name = group_name;
	motion_request.max_velocity_scaling_factor = 1.0;  // TODO: Get from props
	motion_request.max_acceleration_scaling_factor = 1.0;  // TODO: Get from props
	moveit::core::robotStateToRobotStateMsg(sandbox_scene->getCurrentState(), motion_request.start_state);

	// Add goal constraint for target pose
	geometry_msgs::msg::PoseStamped target_msg;
	target_msg.header.frame_id = link_name;
	target_msg.pose = Eigen::toMsg(target);
	motion_request.goal_constraints.resize(1);
	motion_request.goal_constraints[0] = kinematic_constraints::constructGoalConstraints(link_name, target_msg);

	motion_request.path_constraints = path_constraints;

	planning_interface::MotionPlanResponse motion_response;
	const double sampling_time = 0.001;  // TODO Get from props

	const bool success = traj_gen_->generate(sandbox_scene, motion_request, motion_response, sampling_time);
	result = motion_response.trajectory_;
	return success;
}
}  // namespace moveit::task_constructor::solvers
