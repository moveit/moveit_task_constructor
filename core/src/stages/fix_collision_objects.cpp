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
#include <moveit/task_constructor/fmt_p.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/task_constructor/cost_terms.h>

#include <rviz_marker_tools/marker_creation.h>
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif
#include <Eigen/Geometry>
#include <rclcpp/logging.hpp>

namespace vm = visualization_msgs;
namespace cd = collision_detection;

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("FixCollisionObjects");

FixCollisionObjects::FixCollisionObjects(const std::string& name) : PropagatingEitherWay(name) {
	// TODO: possibly weight solutions based on the required displacement?
	setCostTerm(std::make_unique<cost::Constant>(0.0));

	auto& p = properties();
	p.declare<double>("max_penetration", "maximally corrected penetration depth");
	p.declare<geometry_msgs::msg::Vector3>("direction", "direction vector to use for corrections");
}

void FixCollisionObjects::computeForward(const InterfaceState& from) {
	planning_scene::PlanningScenePtr to = from.scene()->diff();
	sendForward(from, InterfaceState(to), fixCollisions(*to));
}

void FixCollisionObjects::computeBackward(const InterfaceState& to) {
	planning_scene::PlanningScenePtr from = to.scene()->diff();
	sendBackward(InterfaceState(from), to, fixCollisions(*from));
}

bool computeCorrection(const std::vector<cd::Contact>& contacts, Eigen::Vector3d& correction,
                       double /*max_penetration*/) {
	correction.setZero();
	for (const cd::Contact& c : contacts) {
		if ((c.body_type_1 != cd::BodyTypes::WORLD_OBJECT && c.body_type_2 != cd::BodyTypes::WORLD_OBJECT)) {
			RCLCPP_WARN_STREAM(LOGGER,
			                   fmt::format("Cannot fix collision between {} and {}", c.body_name_1, c.body_name_2));
			return false;
		}
		if (c.body_type_1 == cd::BodyTypes::WORLD_OBJECT)
			correction -= c.depth * c.normal;
		else
			correction += c.depth * c.normal;
	}
	// average and add tolerance
	double norm = correction.norm();
	double rounded = norm / contacts.size() + 1.e-3;
	correction *= rounded / norm;
	return true;
}

SubTrajectory FixCollisionObjects::fixCollisions(planning_scene::PlanningScene& scene) const {
	SubTrajectory result;
	const auto& props = properties();
	double max_penetration = props.get<double>("max_penetration");
	const boost::any& dir = props.get("direction");

	cd::CollisionRequest req;
	cd::CollisionResult res;
	req.group_name = "";  // check collisions for complete robot
	req.contacts = true;
	req.max_contacts = 100;
	req.max_contacts_per_pair = 100;
	req.verbose = false;
	req.distance = false;

	vm::msg::Marker m;
	m.header.frame_id = scene.getPlanningFrame();
	m.ns = "collisions";

	bool failure = false;
	while (!failure) {
		res.clear();
		scene.getCollisionEnv()->checkRobotCollision(req, res, scene.getCurrentState(),
		                                             scene.getAllowedCollisionMatrix());
		if (!res.collision)
			return result;

		for (const auto& info : res.contacts) {
			Eigen::Vector3d correction;
			failure = !computeCorrection(info.second, correction, max_penetration);
			if (failure)
				break;
			double depth = correction.norm();
			failure = depth > max_penetration;

			// marker indicating correction
			const cd::Contact& c = info.second.front();
			rviz_marker_tools::setColor(m.color, failure ? rviz_marker_tools::RED : rviz_marker_tools::GREEN);
			m.pose = tf2::toMsg(Eigen::Translation3d(c.pos) *
			                    Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), correction));
			rviz_marker_tools::makeArrow(m, depth, true);
			result.markers().push_back(m);
			if (failure)
				break;

			// fix collision by shifting object along correction direction
			if (!dir.empty())  // if explicitly given, use this correction direction
				tf2::fromMsg(boost::any_cast<geometry_msgs::msg::Vector3>(dir), correction);

			const std::string& name = c.body_type_1 == cd::BodyTypes::WORLD_OBJECT ? c.body_name_1 : c.body_name_2;
			scene.getWorldNonConst()->moveObject(name, Eigen::Isometry3d(Eigen::Translation3d(correction)));
		}
	}

	// failure
	result.markAsFailure();
	return result;
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
