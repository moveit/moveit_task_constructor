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

/* Authors: Michael Goerner, Robert Haschke
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
	auto& p = properties();
	p.declare<double>("timeout", 10.0, "planning timeout");
	p.declare<std::string>("group", "name of planning group");
	p.declare<std::string>("link", "", "link to move (default is tip of jmg)");

	p.declare<geometry_msgs::PoseStamped>("pose", "Cartesian target pose");
	p.declare<geometry_msgs::PointStamped>("point", "Cartesian target point");
    p.declare<std::string>("named_joint_pose", "named joint pose");
    p.declare<moveit_msgs::RobotState>("joint_pose", "joint pose as robot state message");

	p.declare<moveit_msgs::Constraints>("path_constraints", moveit_msgs::Constraints(),
	                                    "constraints to maintain during trajectory");
}

void MoveTo::setGroup(const std::string& group){
	setProperty("group", group);
}

void MoveTo::setLink(const std::string& link){
	setProperty("link", link);
}

void MoveTo::setGoal(const geometry_msgs::PoseStamped &pose)
{
	setProperty("pose", pose);
}

void MoveTo::setGoal(const geometry_msgs::PointStamped &point)
{
	setProperty("point", point);
}

void MoveTo::setGoal(const std::string &joint_pose)
{
    setProperty("named_joint_pose", joint_pose);
}

void MoveTo::setGoal(const moveit_msgs::RobotState &robot_state)
{
    setProperty("joint_pose", robot_state);
}

void MoveTo::init(const moveit::core::RobotModelConstPtr& robot_model)
{
    InitStageException errors;

	PropagatingEitherWay::init(robot_model);
	planner_->init(robot_model);

    const auto& props = properties();

    // check if named_joint_pose is set. if so, convert to robot state msg
    boost::any goal = props.get("named_joint_pose");
    if (!goal.empty()) {
        const std::string& named_joint_pose = boost::any_cast<std::string>(goal);
        robot_state::RobotState state(robot_model);

        const std::string& group = props.get<std::string>("group");
        const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(group);

        if (!state.setToDefaultValues(jmg, named_joint_pose)) {
            ROS_WARN("MoveTo: unknown joint pose '%s' for jmg '%s'.", named_joint_pose.c_str(), group.c_str());
            errors.push_back(*this, "MoveTo: unknown joint pose.");
        } else {
            ROS_INFO("MoveTo: setting joint pose '%s' for jmg '%s' as goal.", named_joint_pose.c_str(), group.c_str());

            moveit_msgs::RobotState state_msg;
            robot_state::robotStateToRobotStateMsg(state, state_msg);

            setGoal(state_msg);
        }
    }

    if (errors) throw errors;
}

bool MoveTo::compute(const InterfaceState &state, planning_scene::PlanningScenePtr& scene,
                     SubTrajectory &solution, Direction dir) {
	scene = state.scene()->diff();
	assert(scene->getRobotModel());

	const auto& props = properties();
	double timeout = props.get<double>("timeout");
	const std::string& group = props.get<std::string>("group");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getJointModelGroup(group);
	if (!jmg) {
		ROS_WARN_STREAM("MoveTo: invalid joint model group: " << group);
		return false;
	}

	// only allow single target
    size_t count_goals = props.countDefined({"joint_pose", "pose", "point"});
	if (count_goals != 1) {
		if (count_goals == 0) ROS_WARN("MoveTo: no goal defined");
		else ROS_WARN("MoveTo: cannot plan to multiple goals");
		return false;
	}

	const auto& path_constraints = props.get<moveit_msgs::Constraints>("path_constraints");

	robot_trajectory::RobotTrajectoryPtr robot_trajectory;
	bool success = false;

    boost::any goal = props.get("joint_pose");
    if (!goal.empty()) {
        const moveit_msgs::RobotState& target_state = boost::any_cast<moveit_msgs::RobotState>(goal);
        scene->setCurrentState(target_state);
        success = planner_->plan(state.scene(), scene, jmg, timeout, robot_trajectory, path_constraints);
    } else {
		// Cartesian targets require the link name
		std::string link_name = props.get<std::string>("link");
		const moveit::core::LinkModel* link;
		if (link_name.empty())
			link_name = solvers::getEndEffectorLink(jmg);
		if (link_name.empty() || !(link = scene->getRobotModel()->getLinkModel(link_name))) {
			ROS_WARN_STREAM("MoveTo: no or invalid link name specified: " << link_name);
			return false;
		}

		Eigen::Affine3d target_eigen;

		goal = props.get("pose");
		if (!goal.empty()) {
			const geometry_msgs::PoseStamped& target = boost::any_cast<geometry_msgs::PoseStamped>(goal);
			tf::poseMsgToEigen(target.pose, target_eigen);

			// transform target into global frame
			const Eigen::Affine3d& frame = scene->getFrameTransform(target.header.frame_id);
			target_eigen = frame * target_eigen;

			// frame at target pose
			rviz_marker_tools::appendFrame(solution.markers(), target, 0.1, "ik frame");

			// frame at link
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.header.frame_id = link_name;
			pose_msg.pose.orientation.w = 1.0;
			rviz_marker_tools::appendFrame(solution.markers(), pose_msg, 0.1, "ik frame");
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
			target_eigen = scene->getCurrentState().getGlobalLinkTransform(link_name);
			target_eigen.translation() = target_point;
		}

		success = planner_->plan(state.scene(), *link, target_eigen, jmg, timeout, robot_trajectory, path_constraints);
	}

	// store result
	if (success || (robot_trajectory && storeFailures())) {
		scene->setCurrentState(robot_trajectory->getLastWayPoint());
		if (dir == BACKWARD) robot_trajectory->reverse();
		solution.setTrajectory(robot_trajectory);
	}

	return success;
}

bool MoveTo::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	bool success = compute(from, to, trajectory, FORWARD);
	if (!success) trajectory.setCost(std::numeric_limits<double>::infinity());
	sendForward(from, InterfaceState(to), std::move(trajectory));
	return success;
}

bool MoveTo::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from;
	SubTrajectory trajectory;

	bool success = compute(to, from, trajectory, BACKWARD);
	if (!success) trajectory.setCost(std::numeric_limits<double>::infinity());
	sendBackward(InterfaceState(from), to, std::move(trajectory));
	return success;
}

} } }
