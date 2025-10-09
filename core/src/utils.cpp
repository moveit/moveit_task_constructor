/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner, Robert Haschke */

#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/utils/moveit_error_code.hpp>

#include <moveit/task_constructor/properties.h>
#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
namespace utils {

bool getRobotTipForFrame(const Property& tip_pose, const planning_scene::PlanningScene& scene,
                         const moveit::core::JointModelGroup* jmg, std::string& error_msg,
                         const moveit::core::LinkModel*& robot_link, Eigen::Isometry3d& tip_in_global_frame) {
	auto get_tip = [&jmg]() -> const moveit::core::LinkModel* {
		// determine IK frame from group
		std::vector<const moveit::core::LinkModel*> tips;
		jmg->getEndEffectorTips(tips);
		if (tips.size() != 1) {
			return nullptr;
		}
		return tips[0];
	};

	error_msg = "";

	if (tip_pose.value().empty()) {  // property undefined
		robot_link = get_tip();
		if (!robot_link) {
			error_msg = "missing ik_frame";
			return false;
		}
		tip_in_global_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);
	} else {
		auto ik_pose_msg = boost::any_cast<geometry_msgs::msg::PoseStamped>(tip_pose.value());
		tf2::fromMsg(ik_pose_msg.pose, tip_in_global_frame);

		robot_link = nullptr;
		bool found = false;
		auto ref_frame = scene.getCurrentState().getFrameInfo(ik_pose_msg.header.frame_id, robot_link, found);
		if (!found && !ik_pose_msg.header.frame_id.empty()) {
			std::stringstream ss;
			ss << "ik_frame specified in unknown frame '" << ik_pose_msg.header.frame_id << "'";
			error_msg = ss.str();
			return false;
		}
		if (!robot_link)  // if ik_pose_msg.header.frame_id was empty, use default tip frame
			robot_link = get_tip();
		if (!robot_link) {
			error_msg = "ik_frame doesn't specify a link frame";
			return false;
		} else if (!found) {  // use robot link's frame as reference by default
			ref_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);
		}

		tip_in_global_frame = ref_frame * tip_in_global_frame;
	}

	return true;
}

void addCollisionMarkers(std::vector<visualization_msgs::msg::Marker>& markers, const std::string& frame_id,
                         const collision_detection::CollisionResult::ContactMap& contacts, double radius) {
	visualization_msgs::msg::Marker m;
	m.header.frame_id = frame_id;
	m.ns = "collisions";
	m.type = visualization_msgs::msg::Marker::SPHERE;
	m.action = visualization_msgs::msg::Marker::ADD;
	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;
	m.scale.x = m.scale.y = m.scale.z = radius * 2.0;
	m.color.r = m.color.a = 1.0;

	for (const auto& collision : contacts) {
		for (const auto& contact : collision.second) {
			m.pose.position.x = contact.pos.x();
			m.pose.position.y = contact.pos.y();
			m.pose.position.z = contact.pos.z();
			markers.push_back(m);
		}
	}
}

void addCollisionMarkers(std::vector<visualization_msgs::msg::Marker>& markers,
                         const robot_trajectory::RobotTrajectory& trajectory,
                         const planning_scene::PlanningSceneConstPtr& planning_scene) {
	std::size_t n_wp = trajectory.getWayPointCount();
	for (std::size_t it = 0; it < n_wp; ++it) {
		const moveit::core::RobotState& robot_state = trajectory.getWayPoint(it);

		// Collision contact check
		collision_detection::CollisionRequest req;
		collision_detection::CollisionResult res;
		req.contacts = true;
		req.max_contacts = 10;

		planning_scene->checkCollision(req, res, robot_state);

		if (res.contact_count > 0)
			addCollisionMarkers(markers, planning_scene->getPlanningFrame(), res.contacts);
	}
}

bool hints_at_collisions(const solvers::PlannerInterface::Result& result) {
	static const std::string INVALID_MOTION_PLAN_MSG = []() {
		moveit_msgs::msg::MoveItErrorCodes err;
		err.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_MOTION_PLAN;
		return moveit::core::errorCodeToString(err);
	}();
	return result.message == INVALID_MOTION_PLAN_MSG;
}

}  // namespace utils
}  // namespace task_constructor
}  // namespace moveit
