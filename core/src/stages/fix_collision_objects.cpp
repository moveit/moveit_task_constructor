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

/* Authors: Elham Iravani, Robert Haschke
   Desc:    Fix collisions in input scene
*/

#include <moveit/task_constructor/stages/fix_collision_objects.h>

#include <moveit/task_constructor/storage.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

namespace vm = visualization_msgs;

namespace moveit { namespace task_constructor { namespace stages {

FixCollisionObjects::FixCollisionObjects(const std::string &name)
  : PropagatingEitherWay(name)
{
	auto& p = properties();
	p.declare<double>("max_penetration", "maximally corrected penetration depth");
}

void FixCollisionObjects::setMaxPenetration(double penetration)
{
	setProperty("max_penetration", penetration);
}

void FixCollisionObjects::computeForward(const InterfaceState &from)
{
	planning_scene::PlanningScenePtr to = from.scene()->diff();
	SubTrajectory solution;
	bool success = fixCollisions(*to, solution.markers());
	if (!success) solution.markAsFailure();
	sendForward(from, InterfaceState(to), std::move(solution));
}

void FixCollisionObjects::computeBackward(const InterfaceState &to)
{
	planning_scene::PlanningScenePtr from = to.scene()->diff();
	SubTrajectory solution;
	bool success = fixCollisions(*from, solution.markers());
	if (!success) solution.markAsFailure();
	sendBackward(InterfaceState(from), to, std::move(solution));
}

bool FixCollisionObjects::fixCollisions(planning_scene::PlanningScene &scene, std::deque<visualization_msgs::Marker> &markers) const
{
	const auto& props = properties();
	double penetration = props.get<double>("max_penetration");
	(void) penetration;

	collision_detection::CollisionRequest req;
	collision_detection::CollisionResult res;
	req.group_name = "";  // check collisions for complete robot
	req.contacts = true;
	req.max_contacts = 100;
	req.max_contacts_per_pair = 5;
	req.verbose = false;
	req.distance = false;

	scene.getCollisionWorld()->checkRobotCollision(req, res, *scene.getCollisionRobotUnpadded(), scene.getCurrentState(),
	                                               scene.getAllowedCollisionMatrix());

	geometry_msgs::Pose pose;

	if (!res.collision)
		return true;

	vm::Marker m;
	m.header.frame_id = scene.getPlanningFrame();
	m.ns = "collisions";
	rviz_marker_tools::setColor(m.color, rviz_marker_tools::RED);

	ROS_INFO_THROTTLE(1, "collision(s) detected");
	for (const auto& info : res.contacts) {
		for (const auto& contact : info.second) {
			tf::poseEigenToMsg(Eigen::Translation3d(contact.pos) * Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), contact.normal), m.pose);
			rviz_marker_tools::makeArrow(m, contact.depth);
			markers.push_back(m);
		}
	}

	// check again
	scene.getCollisionWorld()->checkRobotCollision(req, res, *scene.getCollisionRobotUnpadded(), scene.getCurrentState(),
	                                               scene.getAllowedCollisionMatrix());
	return !res.collision;
}

void FixCollisionObjects::fixCollision(planning_scene::PlanningScene &scene, geometry_msgs::Pose pose, const std::string& object) const
{
	moveit_msgs::CollisionObject collision_obj;
	collision_obj.header.frame_id = scene.getPlanningFrame();
	collision_obj.id = object;
	collision_obj.operation = moveit_msgs::CollisionObject::MOVE;

	collision_obj.primitive_poses.resize(1);
	collision_obj.primitive_poses[0] = pose;

	if(!scene.processCollisionObjectMsg(collision_obj))
		std::cout<<"Moving FAILED"<<std::endl;
}

} } }
