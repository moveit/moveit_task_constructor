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

#include <moveit/task_constructor/stages/cartesian_position_motion.h>
#include <moveit/task_constructor/storage.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Geometry>

namespace moveit { namespace task_constructor { namespace stages {

CartesianPositionMotion::CartesianPositionMotion(std::string name)
: PropagatingEitherWay(name),
  step_size_(0.005)
{
}

void CartesianPositionMotion::setGroup(std::string group){
	group_= group;
}

void CartesianPositionMotion::setLink(std::string link){
	link_= link;
}

void CartesianPositionMotion::setMinDistance(double distance){
	min_distance_= distance;
}

void CartesianPositionMotion::setMaxDistance(double distance){
	max_distance_= distance;
}

void CartesianPositionMotion::setMinMaxDistance(double min_distance, double max_distance){
	setMinDistance(min_distance);
	setMaxDistance(max_distance);
}

void CartesianPositionMotion::towards(geometry_msgs::PointStamped towards){
	mode_= CartesianPositionMotion::MODE_TOWARDS;
	towards_= towards;
}

void CartesianPositionMotion::along(geometry_msgs::Vector3Stamped along){
	mode_= CartesianPositionMotion::MODE_ALONG;
	along_= along;
}

void CartesianPositionMotion::setCartesianStepSize(double distance){
	step_size_= distance;
}

namespace {
	bool isValid(planning_scene::PlanningSceneConstPtr scene,
	        robot_state::RobotState* state,
	        const robot_model::JointModelGroup* jmg,
	        const double* joint_positions){
		state->setJointGroupPositions(jmg, joint_positions);
		state->update();
		return !scene->isStateColliding(const_cast<const robot_state::RobotState&>(*state), jmg->getName());
	}
}

bool CartesianPositionMotion::computeForward(const InterfaceState &from){
	planning_scene::PlanningScenePtr result_scene = from.scene()->diff();
	robot_state::RobotState &robot_state = result_scene->getCurrentStateNonConst();

	const moveit::core::JointModelGroup* jmg= robot_state.getJointModelGroup(group_);
	const moveit::core::LinkModel* link_model= robot_state.getRobotModel()->getLinkModel(link_);

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			result_scene,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);

	std::vector<moveit::core::RobotStatePtr> trajectory_steps;
	bool succeeded= false;

	if( mode_ == CartesianPositionMotion::MODE_TOWARDS ){
		const Eigen::Affine3d& frame= from.scene()->getFrameTransform(towards_.header.frame_id);

		const Eigen::Affine3d& link_pose= robot_state.getGlobalLinkTransform(link_);

		Eigen::Vector3d target_point;
		tf::pointMsgToEigen(towards_.point, target_point);
		target_point= frame*target_point;

		// retain orientation of link
		Eigen::Affine3d target= link_pose;
		target.translation()= target_point;

		double achieved_fraction= robot_state.computeCartesianPath(
			jmg,
			trajectory_steps,
			link_model,
			target,
			true, /* global frame */
			step_size_, /* cartesian step size */
			1.5, /* jump threshold */
			is_valid);

		const double achieved_distance= achieved_fraction*(link_pose.translation()-target_point).norm();

		std::cout << "achieved " << achieved_distance << " of cartesian motion" << std::endl;

		succeeded= achieved_distance >= min_distance_;
	}
	else if( mode_ == CartesianPositionMotion::MODE_ALONG ){
		const Eigen::Affine3d& frame= robot_state.getGlobalLinkTransform(along_.header.frame_id);
		Eigen::Vector3d direction;
		tf::vectorMsgToEigen(along_.vector, direction);
		direction= frame.linear()*direction;

		double achieved_distance= robot_state.computeCartesianPath(
			jmg,
			trajectory_steps,
			link_model,
			direction,
			true, /* global frame */
			max_distance_, /* distance */
			step_size_, /* cartesian step size */
			1.5, /* jump threshold */
			is_valid);

		std::cout << "achieved " << achieved_distance << " of cartesian motion" << std::endl;

		succeeded= achieved_distance >= min_distance_;
	}
	else
		throw std::runtime_error("position motion has neither a goal nor a direction");


	if(succeeded){
		robot_trajectory::RobotTrajectoryPtr traj
		      = std::make_shared<robot_trajectory::RobotTrajectory>(robot_state.getRobotModel(), jmg);
		for( auto& tp : trajectory_steps )
			traj->addSuffixWayPoint(tp, 0.0);
		sendForward(from, InterfaceState(result_scene), traj);
	}

	return succeeded;
}

bool CartesianPositionMotion::computeBackward(const InterfaceState &to){
	planning_scene::PlanningScenePtr result_scene = to.scene()->diff();
	robot_state::RobotState &robot_state = result_scene->getCurrentStateNonConst();

	const moveit::core::JointModelGroup* jmg= robot_state.getJointModelGroup(group_);
	const moveit::core::LinkModel* link_model= robot_state.getRobotModel()->getLinkModel(link_);

	const moveit::core::GroupStateValidityCallbackFn is_valid=
		std::bind(
			&isValid,
			result_scene,
			std::placeholders::_1,
			std::placeholders::_2,
			std::placeholders::_3);

	Eigen::Vector3d direction;

	switch(mode_){
	case(CartesianPositionMotion::MODE_TOWARDS):
		{
			const Eigen::Affine3d& link_pose= robot_state.getGlobalLinkTransform(link_);
			direction= link_pose.linear()*Eigen::Vector3d(-1,0,0);
		}
		break;
	case(CartesianPositionMotion::MODE_ALONG):
		{
			const Eigen::Affine3d& frame= robot_state.getGlobalLinkTransform(along_.header.frame_id);
			tf::vectorMsgToEigen(along_.vector, direction);
			direction= frame.linear()*direction;
		}
		break;
	default:
		throw std::runtime_error("position motion has neither a goal nor a direction");
	}

	std::vector<moveit::core::RobotStatePtr> trajectory_steps;

	double achieved_distance= robot_state.computeCartesianPath(
		jmg,
		trajectory_steps,
		link_model,
		direction,
		true, /* global frame */
		max_distance_, /* distance */
		step_size_, /* cartesian step size */
		1.5, /* jump threshold */
		is_valid);

	std::cout << "achieved " << achieved_distance << " of cartesian motion" << std::endl;

	bool succeeded= achieved_distance >= min_distance_;

	if(succeeded){
		robot_trajectory::RobotTrajectoryPtr traj= std::make_shared<robot_trajectory::RobotTrajectory>(robot_state.getRobotModel(), jmg);
		for( auto& tp : trajectory_steps )
			traj->addPrefixWayPoint(tp, 0.0);
		sendBackward(InterfaceState(result_scene), to, traj);
	}

	return succeeded;
}

} } }
