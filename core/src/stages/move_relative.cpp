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

/* Authors: Robert Haschke, Michael Goerner
   Desc:    Move link along Cartesian direction
*/

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("MoveRelative");

MoveRelative::MoveRelative(const std::string& name, const solvers::PlannerInterfacePtr& planner)
  : PropagatingEitherWay(name), planner_(planner) {
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	p.property("timeout").setDefaultValue(1.0);
	p.declare<std::string>("group", "name of planning group");
	p.declare<geometry_msgs::msg::PoseStamped>("ik_frame", "frame to be moved in Cartesian direction");

	p.declare<boost::any>("direction", "motion specification");
	// register actual types
	PropertySerializer<geometry_msgs::msg::TwistStamped>();
	PropertySerializer<geometry_msgs::msg::Vector3Stamped>();
	p.declare<double>("min_distance", -1.0, "minimum distance to move");
	p.declare<double>("max_distance", 0.0, "maximum distance to move");

	p.declare<moveit_msgs::msg::Constraints>("path_constraints", moveit_msgs::msg::Constraints(),
	                                         "constraints to maintain during trajectory");
}

void MoveRelative::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::msg::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	pose_msg.pose = tf2::toMsg(pose);
	setIKFrame(pose_msg);
}

void MoveRelative::init(const moveit::core::RobotModelConstPtr& robot_model) {
	PropagatingEitherWay::init(robot_model);
	planner_->init(robot_model);
}

static bool getJointStateFromOffset(const boost::any& direction, const Interface::Direction dir,
                                    const moveit::core::JointModelGroup* jmg, moveit::core::RobotState& robot_state) {
	try {
		const auto& accepted = jmg->getActiveJointModels();
		const auto& joints = boost::any_cast<std::map<std::string, double>>(direction);
		double sign = dir == Interface::Direction::FORWARD ? +1.0 : -1.0;
		for (const auto& j : joints) {
			auto jm = robot_state.getRobotModel()->getJointModel(j.first);
			if (!jm || std::find(accepted.begin(), accepted.end(), jm) == accepted.end())
				throw std::runtime_error("Cannot plan for joint '" + j.first + "' that is not part of group '" +
				                         jmg->getName() + "'");
			if (jm->getVariableCount() != 1)
				throw std::runtime_error("Cannot plan for multi-variable joint '" + j.first + "'");
			auto index = jm->getFirstVariableIndex();
			robot_state.setVariablePosition(index, robot_state.getVariablePosition(index) + sign * j.second);
			robot_state.enforceBounds(jm);
		}
		robot_state.update();
		return true;
	} catch (const boost::bad_any_cast&) {
		return false;
	}

	return false;
}

// Create an arrow marker from start_pose to reached_pose, split into a red and green part based on achieved distance
static void visualizePlan(std::deque<visualization_msgs::msg::Marker>& markers, Interface::Direction dir, bool success,
                          const std::string& ns, const std::string& frame_id, const Eigen::Isometry3d& start_pose,
                          const Eigen::Isometry3d& reached_pose, const Eigen::Vector3d& linear, double distance) {
	double linear_norm = linear.norm();

	// rotation of the target direction and for the cylinder marker
	auto quat_target = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), linear);
	auto quat_cylinder = quat_target * Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());

	// link position before planning; reached link position after planning; target link position
	Eigen::Vector3d pos_start = start_pose.translation();
	Eigen::Vector3d pos_reached = reached_pose.translation();
	Eigen::Vector3d pos_target = pos_reached + quat_target * Eigen::Vector3d(linear_norm - distance, 0, 0);

	visualization_msgs::msg::Marker m;
	m.ns = ns;
	m.header.frame_id = frame_id;
	if (dir == Interface::FORWARD) {
		if (success) {
			// valid part: green arrow
			rviz_marker_tools::makeArrow(m, pos_start, pos_reached, 0.1 * linear_norm);
			rviz_marker_tools::setColor(m.color, rviz_marker_tools::LIME_GREEN);
			markers.push_back(m);
		} else {
			// invalid part: red arrow
			// set head length to keep default shaft:head proportion of 1:0.3 as defined in
			// rviz/default_plugin/markers/arrow_marker.cpp#L105
			rviz_marker_tools::makeArrow(m, pos_reached, pos_target, 0.1 * linear_norm, 0.23 * linear_norm);
			rviz_marker_tools::setColor(m.color, rviz_marker_tools::RED);
			markers.push_back(m);

			// valid part: green cylinder
			rviz_marker_tools::makeCylinder(m, 0.1 * linear_norm, distance);
			rviz_marker_tools::setColor(m.color, rviz_marker_tools::LIME_GREEN);
			// position half-way between pos_link and pos_reached
			m.pose.position = tf2::toMsg(Eigen::Vector3d{ 0.5 * (pos_start + pos_reached) });
			m.pose.orientation = tf2::toMsg(quat_cylinder);
			markers.push_back(m);
		}
	} else {
		// valid part: green arrow
		// head length according to above comment
		rviz_marker_tools::makeArrow(m, pos_reached, pos_start, 0.1 * linear_norm, 0.23 * linear_norm);
		rviz_marker_tools::setColor(m.color, rviz_marker_tools::LIME_GREEN);
		markers.push_back(m);
		if (!success) {
			// invalid part: red cylinder
			rviz_marker_tools::makeCylinder(m, 0.1 * linear_norm, linear_norm - distance);
			rviz_marker_tools::setColor(m.color, rviz_marker_tools::RED);
			// position half-way between pos_reached and pos_target
			m.pose.position = tf2::toMsg(Eigen::Vector3d{ 0.5 * (pos_reached + pos_target) });
			m.pose.orientation = tf2::toMsg(quat_cylinder);
			markers.push_back(m);
		}
	}
}

bool MoveRelative::compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene,
                           SubTrajectory& solution, Interface::Direction dir) {
	scene = state.scene()->diff();
	const moveit::core::RobotModelConstPtr& robot_model = scene->getRobotModel();
	assert(robot_model);

	const auto& props = properties();
	double timeout = this->timeout();
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
	if (!jmg) {
		solution.markAsFailure("invalid joint model group: " + group);
		return false;
	}
	boost::any direction = props.get("direction");
	if (direction.empty()) {
		solution.markAsFailure("undefined direction");
		return false;
	}

	double max_distance = props.get<double>("max_distance");
	double min_distance = props.get<double>("min_distance");

	// if min_distance == max_distance, resort to moving full distance (negative min_distance)
	if (max_distance > 0.0 && std::fabs(max_distance - min_distance) < std::numeric_limits<double>::epsilon())
		min_distance = -1.0;

	const auto& path_constraints = props.get<moveit_msgs::msg::Constraints>("path_constraints");

	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;
	std::string comment = "";

	if (getJointStateFromOffset(direction, dir, jmg, scene->getCurrentStateNonConst())) {
		// plan to joint-space target
		auto result = planner_->plan(state.scene(), scene, jmg, timeout, robot_trajectory, path_constraints);
		success = bool(result);
		if (!success)
			comment = result.message;
		solution.setPlannerId(planner_->getPlannerId());
	} else {
		// Cartesian targets require an IK reference frame
		const moveit::core::LinkModel* link;
		std::string error_msg;
		Eigen::Isometry3d ik_pose_world;

		if (!utils::getRobotTipForFrame(props.property("ik_frame"), *scene, jmg, error_msg, link, ik_pose_world)) {
			solution.markAsFailure(error_msg);
			return false;
		}

		bool use_rotation_distance = false;  // measure achieved distance as rotation?
		Eigen::Vector3d linear;  // linear translation
		Eigen::Vector3d angular;  // angular rotation
		double linear_norm = 0.0, angular_norm = 0.0;
		Eigen::Isometry3d target_eigen;

		try {  // try to extract Twist
			const geometry_msgs::msg::TwistStamped& target = boost::any_cast<geometry_msgs::msg::TwistStamped>(direction);
			const Eigen::Isometry3d& frame_pose = scene->getFrameTransform(target.header.frame_id);
			tf2::fromMsg(target.twist.linear, linear);
			tf2::fromMsg(target.twist.angular, angular);

			linear_norm = linear.norm();
			angular_norm = angular.norm();
			if (angular_norm > std::numeric_limits<double>::epsilon())
				angular /= angular_norm;  // normalize angular
			use_rotation_distance = linear_norm < std::numeric_limits<double>::epsilon();

			// use max distance?
			if (max_distance > 0.0) {
				double scale = 1.0;
				if (!use_rotation_distance)  // non-zero linear motion defines distance
					scale = max_distance / linear_norm;
				else if (angular_norm > std::numeric_limits<double>::epsilon())
					scale = max_distance / angular_norm;
				else
					assert(false);
				linear *= scale;
				linear_norm *= scale;
				angular_norm *= scale;
			}

			// invert direction?
			if (dir == Interface::BACKWARD) {
				linear *= -1.0;
				angular *= -1.0;
			}

			// compute target transform for ik_frame applying motion transform of twist
			// linear+angular are expressed w.r.t. model frame and thus we need left-multiplication
			linear = frame_pose.linear() * linear;
			angular = frame_pose.linear() * angular;
			auto R = Eigen::AngleAxisd(angular_norm, angular);  // NOLINT(readability-identifier-naming)
			auto p = ik_pose_world.translation();
			target_eigen = Eigen::Translation3d(linear + p - R * p) * (R * ik_pose_world);
			goto COMPUTE;
		} catch (const boost::bad_any_cast&) { /* continue with Vector */
		}

		try {  // try to extract Vector
			const geometry_msgs::msg::Vector3Stamped& target =
			    boost::any_cast<geometry_msgs::msg::Vector3Stamped>(direction);
			const Eigen::Isometry3d& frame_pose = scene->getFrameTransform(target.header.frame_id);
			tf2::fromMsg(target.vector, linear);

			// use max distance?
			if (max_distance > 0.0) {
				linear.normalize();
				linear *= max_distance;
			}
			linear_norm = linear.norm();

			// invert direction?
			if (dir == Interface::BACKWARD)
				linear *= -1.0;

			// compute target transform for ik_frame applying delta transform of twist
			linear = frame_pose.linear() * linear;
			target_eigen = Eigen::Translation3d(linear) * ik_pose_world;
		} catch (const boost::bad_any_cast&) {
			solution.markAsFailure(std::string("invalid direction type: ") + direction.type().name());
			return false;
		}

	COMPUTE:
		// offset from link to ik_frame
		const Eigen::Isometry3d& offset = scene->getCurrentState().getGlobalLinkTransform(link).inverse() * ik_pose_world;

		auto result =
		    planner_->plan(state.scene(), *link, offset, target_eigen, jmg, timeout, robot_trajectory, path_constraints);
		success = bool(result);
		if (!success)
			comment = result.message;
		solution.setPlannerId(planner_->getPlannerId());

		if (robot_trajectory && robot_trajectory->getWayPointCount() > 0) {  // the following requires a robot_trajectory
			                                                                  // returned from planning
			moveit::core::RobotStatePtr& reached_state = robot_trajectory->getLastWayPointPtr();
			reached_state->updateLinkTransforms();
			const Eigen::Isometry3d& reached_pose = reached_state->getGlobalLinkTransform(link) * offset;

			double distance = 0.0;
			if (use_rotation_distance) {
				Eigen::AngleAxisd rotation(reached_pose.linear() * ik_pose_world.linear().transpose());
				distance = rotation.angle();
			} else
				distance = (reached_pose.translation() - ik_pose_world.translation()).norm();

			// min_distance reached?
			if (min_distance > 0.0) {
				success = distance >= min_distance;
				if (!success) {
					char msg[100];
					snprintf(msg, sizeof(msg), "min_distance not reached (%.3g < %.3g)", distance, min_distance);
					solution.setComment(msg);
				}
			} else if (min_distance == 0.0) {  // if min_distance is zero, we succeed in any case
				success = true;
			} else if (!success)
				solution.setComment("failed to move full distance");

			// visualize plan
			auto ns = props.get<std::string>("marker_ns");
			if (!ns.empty() && linear_norm > 0) {  // ensures that 'distance' is the norm of the reached distance
				visualizePlan(solution.markers(), dir, success, ns, scene->getPlanningFrame(), ik_pose_world, reached_pose,
				              linear, distance);
			}
		}
	}

	// store result
	if (robot_trajectory && robot_trajectory->getWayPointCount() > 0) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == Interface::BACKWARD)
			robot_trajectory->reverse();
		solution.setTrajectory(robot_trajectory);

		if (!success)
			solution.markAsFailure(comment);
		return true;
	}
	return false;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
