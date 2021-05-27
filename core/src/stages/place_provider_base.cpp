/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

/* Authors: Artur Karoly, Jafar Abdi */

#include <pluginlib/class_list_macros.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <moveit/robot_state/attached_body.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/stages/place_provider.h>
#include "moveit/task_constructor/stages/place_provider_base.h"

#include <moveit_msgs/PlaceLocation.h>

namespace moveit {
namespace task_constructor {
namespace stages {
PlaceProviderBase::PlaceProviderBase(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("object");
	p.declare<::geometry_msgs::PoseStamped_<std::allocator<void>>>("ik_frame");
	p.declare<std::vector<moveit_msgs::PlaceLocation>>("place_locations");
}
void PlaceProviderBase::onNewSolution(const SolutionBase& s) {
	std::shared_ptr<const planning_scene::PlanningScene> scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	std::string msg;
	if (!scene->getCurrentState().hasAttachedBody(object))
		msg = "'" + object + "' is not an attached object";
	if (scene->getCurrentState().getAttachedBody(object)->getFixedTransforms().empty())
		msg = "'" + object + "' has no associated shapes";
	if (!msg.empty()) {
		if (storeFailures()) {
			InterfaceState state(scene);
			SubTrajectory solution;
			solution.markAsFailure();
			solution.setComment(msg);
			spawn(std::move(state), std::move(solution));
		} else
			ROS_WARN_STREAM_NAMED("PlaceProviderBase", msg);
		return;
	}

	upstream_solutions_.push(&s);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
