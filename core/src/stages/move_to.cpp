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
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_state/conversions.h>

namespace moveit { namespace task_constructor { namespace stages {

MoveTo::MoveTo(const std::string& name, const solvers::PlannerInterfacePtr& planner)
   : PropagatingEitherWay(name)
   , planner_(planner)
{
	setTimeout(10.0);
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");
	p.declare<geometry_msgs::PoseStamped>("ik_frame", "frame to be moved towards goal pose");

	p.declare<geometry_msgs::PoseStamped>("pose", "Cartesian target pose");
	p.declare<geometry_msgs::PointStamped>("point", "Cartesian target point");
	p.declare<std::string>("named_joint_pose", "named joint pose");
	p.declare<moveit_msgs::RobotState>("joint_pose", "joint pose");

	p.declare<moveit_msgs::Constraints>("path_constraints", moveit_msgs::Constraints(),
	                                    "constraints to maintain during trajectory");
}

void MoveTo::setIKFrame(const Eigen::Affine3d& pose, const std::string& link)
{
	geometry_msgs::PoseStamped pose_msg;
	pose_msg.header.frame_id = link;
	tf::poseEigenToMsg(pose, pose_msg.pose);
	setIKFrame(pose_msg);
}

void MoveTo::init(const moveit::core::RobotModelConstPtr& robot_model)
{
	PropagatingEitherWay::init(robot_model);
	planner_->init(robot_model);

	// Check if named_joint_pose is well-defined in JMG.
	robot_state::RobotState state(robot_model);
	getJointStateGoal(state);
}

bool MoveTo::getJointStateGoal(moveit::core::RobotState& state) {
	const auto& props = properties();
	if (props.countDefined({"named_joint_pose", "joint_pose"}) == 2)
		throw InitStageException(*this, "Cannot define both, named_joint_pose and joint_pose");

	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = state.getJointModelGroup(group);
	if (!jmg)
		throw InitStageException(*this, "Unknown joint model group: " + group);

	boost::any goal = props.get("named_joint_pose");
	if (!goal.empty()) {
		const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
		if (!state.setToDefaultValues(jmg, named_joint_pose))
			throw InitStageException(*this, "Unknown joint pose: " + named_joint_pose);
		return true;
	}

	goal = props.get("joint_pose");
	if (!goal.empty()) {
		const moveit_msgs::RobotState& msg = boost::any_cast<moveit_msgs::RobotState>(goal);
		if (!msg.is_diff)
			throw InitStageException(*this, "Expecting a diff state");

		// validate specified joints
		const auto& accepted = jmg->getJointModelNames();
		for (const auto &name : msg.joint_state.name)
			if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
				throw InitStageException(*this, "Joint '" + name + "' is not part of group '" + group + "'");
		for (const auto &name : msg.multi_dof_joint_state.joint_names)
			if (std::find(accepted.begin(), accepted.end(), name) == accepted.end())
				throw InitStageException(*this, "Joint '" + name + "' is not part of group '" + group + "'");

		moveit::core::robotStateMsgToRobotState(msg, state, false);
		return true;
	}
	return false;
}

bool MoveTo::compute(const InterfaceState &state, planning_scene::PlanningScenePtr& scene,
                     SubTrajectory &solution, Direction dir) {
	scene = state.scene()->diff();
	const robot_model::RobotModelConstPtr& robot_model = scene->getRobotModel();
	assert(robot_model);

	const auto& props = properties();
	double timeout = this->timeout();
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);
	if (!jmg) {
		ROS_WARN_STREAM_NAMED("MoveTo", "Invalid joint model group: " << group);
		return false;
	}

	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");
	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;

	bool has_joint_state_goal;
	try {
		has_joint_state_goal = getJointStateGoal(scene->getCurrentStateNonConst());
	} catch (const InitStageException &e) {
		ROS_WARN_STREAM_NAMED("MoveTo", e.what());
		return false;
	}

	size_t cartesian_goals = props.countDefined({"pose", "point"});

	if (cartesian_goals > 1) {
		ROS_WARN_NAMED("MoveTo", "Ambiguous goals: Multiple Cartesian goals defined");
		return false;
	}

	if (cartesian_goals > 0 && has_joint_state_goal) {
		ROS_WARN_NAMED("MoveTo", "Ambiguous goals: Cartesian goals and joint state goals defined");
		return false;
	}

	if (cartesian_goals == 0 && !has_joint_state_goal) {
		ROS_WARN_NAMED("MoveTo", "No goal defined");
		return false;
	}

	if (has_joint_state_goal) {
		// plan to joint-space target
		success = planner_->plan(state.scene(), scene, jmg, timeout, robot_trajectory, path_constraints);
	} else if (cartesian_goals == 1) {
		const moveit::core::LinkModel* link;
		Eigen::Affine3d target_eigen;

		// Cartesian targets require an IK reference frame
		geometry_msgs::PoseStamped ik_pose_msg;
		const boost::any& value = props.get("ik_frame");
		if (value.empty()) { // property undefined
			// determine IK link from group
			if (!(link = jmg->getOnlyOneEndEffectorTip())) {
				ROS_WARN_STREAM_NAMED("MoveTo", "Failed to derive IK target link");
				return false;
			}
			ik_pose_msg.header.frame_id = link->getName();
			ik_pose_msg.pose.orientation.w = 1.0;
		} else {
			ik_pose_msg = boost::any_cast<geometry_msgs::PoseStamped>(value);
			if (!(link = robot_model->getLinkModel(ik_pose_msg.header.frame_id))) {
				ROS_WARN_STREAM_NAMED("MoveTo", "Unknown link: " << ik_pose_msg.header.frame_id);
				return false;
			}
		}

		boost::any goal = props.get("pose");
		if (!goal.empty()) {
			const geometry_msgs::PoseStamped& target = boost::any_cast<geometry_msgs::PoseStamped>(goal);
			tf::poseMsgToEigen(target.pose, target_eigen);

			// transform target into global frame
			const Eigen::Affine3d& frame = scene->getFrameTransform(target.header.frame_id);
			target_eigen = frame * target_eigen;

			// frame at target pose
			rviz_marker_tools::appendFrame(solution.markers(), target, 0.1, "ik frame");

			// frame at link
			rviz_marker_tools::appendFrame(solution.markers(), ik_pose_msg, 0.1, "ik frame");
		}

		goal = props.get("point");
		if (!goal.empty()) {
			const geometry_msgs::PointStamped& target = boost::any_cast<geometry_msgs::PointStamped>(goal);
			Eigen::Vector3d target_point;
			tf::pointMsgToEigen(target.point, target_point);

			// transform target into global frame
			const Eigen::Affine3d& frame = scene->getFrameTransform(target.header.frame_id);
			target_point = frame * target_point;

			// retain link orientation
			target_eigen = scene->getCurrentState().getGlobalLinkTransform(link);
			target_eigen.translation() = target_point;
		}

		// transform target pose such that ik frame will reach there if link does
		Eigen::Affine3d ik_pose;
		tf::poseMsgToEigen(ik_pose_msg.pose, ik_pose);
		target_eigen = target_eigen * ik_pose.inverse();

		// plan to Cartesian target
		success = planner_->plan(state.scene(), *link, target_eigen, jmg, timeout, robot_trajectory, path_constraints);
	}

	// store result
	if (robot_trajectory) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == BACKWARD) robot_trajectory->reverse();
		solution.setTrajectory(robot_trajectory);
		if (!success)
			solution.markAsFailure();
	}
	return true;
}

// -1 TODO: move these as default implementation to PropagateEitherWay?
// Essentially, here compute() is a class-specific worker function
void MoveTo::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	if (compute(from, to, trajectory, FORWARD))
		sendForward(from, InterfaceState(to), std::move(trajectory));
	else
		silentFailure();
}

void MoveTo::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from;
	SubTrajectory trajectory;

	if (compute(to, from, trajectory, BACKWARD))
		sendBackward(InterfaceState(from), to, std::move(trajectory));
	else
		silentFailure();
}

} } }
