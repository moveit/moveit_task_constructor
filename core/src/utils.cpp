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

#include <tf2_eigen/tf2_eigen.h>

#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/task_constructor/moveit_compat.h>
#include <moveit/task_constructor/properties.h>
#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
namespace utils {

const moveit::core::LinkModel* getRigidlyConnectedParentLinkModel(const moveit::core::RobotState& state,
                                                                  std::string frame) {
#if MOVEIT_HAS_STATE_RIGID_PARENT_LINK
	return state.getRigidlyConnectedParentLinkModel(frame);
#else
	const moveit::core::LinkModel* link{ nullptr };

	if (state.hasAttachedBody(frame)) {
		link = state.getAttachedBody(frame)->getAttachedLink();
	} else if (state.getRobotModel()->hasLinkModel(frame))
		link = state.getLinkModel(frame);

	return state.getRobotModel()->getRigidlyConnectedParentLinkModel(link);
#endif
}

bool getRobotTipForFrame(const Property& property, const planning_scene::PlanningScene& scene,
                         const moveit::core::JointModelGroup* jmg, SolutionBase& solution,
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

	if (property.value().empty()) {  // property undefined
		robot_link = get_tip();
		if (!robot_link) {
			solution.markAsFailure("missing ik_frame");
			return false;
		}
		tip_in_global_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link);
	} else {
		auto ik_pose_msg = boost::any_cast<geometry_msgs::PoseStamped>(property.value());
		if (ik_pose_msg.header.frame_id.empty()) {
			if (!(robot_link = get_tip())) {
				solution.markAsFailure("frame_id of ik_frame is empty and no unique group tip was found");
				return false;
			}
			tf2::fromMsg(ik_pose_msg.pose, tip_in_global_frame);
			tip_in_global_frame = scene.getCurrentState().getGlobalLinkTransform(robot_link) * tip_in_global_frame;
		} else if (scene.knowsFrameTransform(ik_pose_msg.header.frame_id)) {
			robot_link = getRigidlyConnectedParentLinkModel(scene.getCurrentState(), ik_pose_msg.header.frame_id);
			tf2::fromMsg(ik_pose_msg.pose, tip_in_global_frame);
			tip_in_global_frame = scene.getFrameTransform(ik_pose_msg.header.frame_id) * tip_in_global_frame;
		} else {
			std::stringstream ss;
			ss << "ik_frame specified in unknown frame '" << ik_pose_msg << "'";
			solution.markAsFailure(ss.str());
			return false;
		}
	}

	return true;
}

}  // namespace utils
}  // namespace task_constructor
}  // namespace moveit
