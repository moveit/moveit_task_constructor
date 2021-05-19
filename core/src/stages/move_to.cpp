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
   Desc:    Move to joint-state or Cartesian goal pose
*/

#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

namespace moveit {
namespace task_constructor {
namespace stages {

MoveTo::MoveTo(const std::string& name, const solvers::PlannerInterfacePtr& planner)
  : PropagatingEitherWay(name), planner_(planner) {
	setCostTerm(std::make_unique<cost::PathLength>());

	auto& p = properties();
	p.property("timeout").setDefaultValue(1.0);
	p.declare<std::string>("group", "name of planning group");
	p.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose");
	p.declare<boost::any>("goal", "goal specification");
	// register actual types
	PropertySerializer<std::string>();
	PropertySerializer<moveit_msgs::RobotState>();
	PropertySerializer<geometry_msgs::PointStamped>();
	PropertySerializer<geometry_msgs::PoseStamped>();

	p.declare<moveit_msgs::Constraints>("path_constraints", moveit_msgs::Constraints(),
	                                    "constraints to maintain during trajectory");
}

void MoveTo::setIKFrame(const Eigen::Isometry3d& pose, const std::string& link) {
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setIKFrame(pose_msg);
}

void MoveTo::setGoal(const std::map<std::string, double>& joints) {
	moveit_msgs::RobotState robot_state;
	robot_state.joint_state.name.reserve(joints.size());
	robot_state.joint_state.position.reserve(joints.size());

	for (auto& joint : joints) {
		robot_state.joint_state.name.push_back(joint.first);
		robot_state.joint_state.position.push_back(joint.second);
	}
	robot_state.is_diff = true;
	setProperty("goal", robot_state);
}

void MoveTo::init(const moveit::core::RobotModelConstPtr& robot_model) {
	PropagatingEitherWay::init(robot_model);
	planner_->init(robot_model);
}

bool MoveTo::getJointStateGoal(const boost::any& goal, const moveit::core::JointModelGroup* jmg,
                               moveit::core::RobotState& state) {
	try {
		// try named joint pose
		const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
		if (!state.setToDefaultValues(jmg, named_joint_pose))
			throw InitStageException(*this, "Unknown joint pose: " + named_joint_pose);
		state.update();
		return true;
	} catch (const boost::bad_any_cast&) {
	}

	try {
		// try RobotState
		const moveit_msgs::RobotState& msg = boost::any_cast<moveit_msgs::RobotState>(goal);
		if (!msg.is_diff)
			throw InitStageException(*this, "Expecting a diff state");

		// validate specified joints
		const auto& accepted = jmg->getJointModelNames();
		for (const auto& name : msg.joint_state.name)
			if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
				throw InitStageException(*this, "Joint '" + name + "' is not part of group '" + jmg->getName() + "'");
		for (const auto& name : msg.multi_dof_joint_state.joint_names)
			if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
				throw InitStageException(*this, "Joint '" + name + "' is not part of group '" + jmg->getName() + "'");

		moveit::core::robotStateMsgToRobotState(msg, state, false);
		return true;
	} catch (const boost::bad_any_cast&) {
	}

	try {
		const std::map<std::string, double>& joint_map = boost::any_cast<std::map<std::string, double>>(goal);
		const auto& accepted = jmg->getJointModelNames();
		for (const auto& joint : joint_map) {
			if (std::find(accepted.begin(), accepted.end(), joint.first) == accepted.end())
				throw InitStageException(*this,
				                         "Joint '" + joint.first + "' is not part of group '" + jmg->getName() + "'");
			state.setVariablePosition(joint.first, joint.second);
		}
		state.update();
		return true;
	} catch (const boost::bad_any_cast&) {
	}

	return false;
}

bool MoveTo::getPoseGoal(const boost::any& goal, const planning_scene::PlanningScenePtr& scene,
                         Eigen::Isometry3d& target_eigen) {
	try {
		const geometry_msgs::PoseStamped& target = boost::any_cast<geometry_msgs::PoseStamped>(goal);
		tf::poseMsgToEigen(target.pose, target_eigen);

		// transform target into global frame
		const Eigen::Isometry3d& frame = scene->getFrameTransform(target.header.frame_id);
		target_eigen = frame * target_eigen;
	} catch (const boost::bad_any_cast&) {
		return false;
	}
	return true;
}

bool MoveTo::getPointGoal(const boost::any& goal, const moveit::core::LinkModel* link,
                          const planning_scene::PlanningScenePtr& scene, Eigen::Isometry3d& target_eigen) {
	try {
		const geometry_msgs::PointStamped& target = boost::any_cast<geometry_msgs::PointStamped>(goal);
		Eigen::Vector3d target_point;
		tf::pointMsgToEigen(target.point, target_point);

		// transform target into global frame
		const Eigen::Isometry3d& frame = scene->getFrameTransform(target.header.frame_id);
		target_point = frame * target_point;

		// retain link orientation
		target_eigen = scene->getCurrentState().getGlobalLinkTransform(link);
		target_eigen.translation() = target_point;
	} catch (const boost::bad_any_cast&) {
		return false;
	}
	return true;
}

bool MoveTo::compute(const InterfaceState& state, planning_scene::PlanningScenePtr& scene, SubTrajectory& solution,
                     Interface::Direction dir) {
	scene = state.scene()->diff();
	const robot_model::RobotModelConstPtr& robot_model = scene->getRobotModel();
	assert(robot_model);

	const auto& props = properties();
	double timeout = this->timeout();
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
	if (!jmg) {
		solution.markAsFailure("invalid joint model group: " + group);
		return false;
	}
	boost::any goal = props.get("goal");
	if (goal.empty()) {
		solution.markAsFailure("undefined goal");
		return false;
	}

	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");
	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;

	if (getJointStateGoal(goal, jmg, scene->getCurrentStateNonConst())) {
		// plan to joint-space target
		success = planner_->plan(state.scene(), scene, jmg, timeout, robot_trajectory, path_constraints);
	} else {  // Cartesian goal
		const moveit::core::LinkModel* link;
		Eigen::Isometry3d target_eigen;

		// Cartesian targets require an IK reference frame
		geometry_msgs::PoseStamped ik_pose_msg;
		const boost::any& value = props.get("ik_frame");
		if (value.empty()) {  // property undefined
			// determine IK link from group
			if (!(link = jmg->getOnlyOneEndEffectorTip())) {
				solution.markAsFailure("missing ik_frame");
				return false;
			}
			ik_pose_msg.header.frame_id = link->getName();
			ik_pose_msg.pose.orientation.w = 1.0;
		} else {
			ik_pose_msg = boost::any_cast<geometry_msgs::PoseStamped>(value);
			if (!(link = robot_model->getLinkModel(ik_pose_msg.header.frame_id))) {
				solution.markAsFailure("unknown link for ik_frame: " + ik_pose_msg.header.frame_id);
				return false;
			}
		}

		if (!getPoseGoal(goal, scene, target_eigen) && !getPointGoal(goal, link, scene, target_eigen)) {
			solution.markAsFailure(std::string("invalid goal type: ") + goal.type().name());
			return false;
		}

		// visualize plan with frame at target pose and frame at link
		geometry_msgs::PoseStamped target;
		target.header.frame_id = scene->getPlanningFrame();
		tf::poseEigenToMsg(target_eigen, target.pose);
		rviz_marker_tools::appendFrame(solution.markers(), target, 0.1, "ik frame");
		rviz_marker_tools::appendFrame(solution.markers(), ik_pose_msg, 0.1, "ik frame");

		// transform target pose such that ik frame will reach there if link does
		Eigen::Isometry3d ik_pose;
		tf::poseMsgToEigen(ik_pose_msg.pose, ik_pose);
		target_eigen = target_eigen * ik_pose.inverse();

		// plan to Cartesian target
		success = planner_->plan(state.scene(), *link, target_eigen, jmg, timeout, robot_trajectory, path_constraints);
	}

	// store result
	if (!robot_trajectory && storeFailures()) {
		robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, jmg);
		robot_trajectory->addSuffixWayPoint(state.scene()->getCurrentState(), 0.0);
		robot_trajectory->addSuffixWayPoint(scene->getCurrentState(), 1.0);
	}
	if (robot_trajectory) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == Interface::BACKWARD)
			robot_trajectory->reverse();
		solution.setTrajectory(robot_trajectory);

		if (!success)
			solution.markAsFailure();

		return true;
	}
	return false;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
