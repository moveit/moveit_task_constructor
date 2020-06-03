/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Bielefeld University
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

/* Authors: Robert Haschke */

#include <moveit/task_constructor/stages/fixed_cartesian_poses.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/marker_tools.h>

#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>

namespace moveit {
namespace task_constructor {
namespace stages {

static const rclcpp::Logger LOGGER = rclcpp::get_logger("FixedCartesianPoses");

using PosesList = std::vector<geometry_msgs::msg::PoseStamped>;

FixedCartesianPoses::FixedCartesianPoses(const std::string& name) : MonitoringGenerator(name) {
	setCostTerm(std::make_unique<cost::Constant>(0.0));

	auto& p = properties();
	p.declare<PosesList>("poses", PosesList(), "target poses to spawn");
}

void FixedCartesianPoses::addPose(const geometry_msgs::msg::PoseStamped& pose) {
	moveit::task_constructor::Property& poses = properties().property("poses");
	if (!poses.defined())
		poses.setValue(PosesList({ pose }));
	else
		boost::any_cast<PosesList&>(poses.value()).push_back(pose);
}

void FixedCartesianPoses::reset() {
	upstream_solutions_.clear();
	MonitoringGenerator::reset();
}

void FixedCartesianPoses::onNewSolution(const SolutionBase& s) {
	// It's safe to store a pointer to this solution, as the generating stage stores it
	upstream_solutions_.push(&s);
}

bool FixedCartesianPoses::canCompute() const {
	return !upstream_solutions_.empty();
}

void FixedCartesianPoses::compute() {
	if (upstream_solutions_.empty())
		return;

	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();
	for (geometry_msgs::msg::PoseStamped pose : properties().get<PosesList>("poses")) {
		if (pose.header.frame_id.empty())
			pose.header.frame_id = scene->getPlanningFrame();
		else if (!scene->knowsFrameTransform(pose.header.frame_id)) {
			RCLCPP_WARN(LOGGER, "Unknown frame: '%s'", pose.header.frame_id.c_str());
			continue;
		}

		InterfaceState state(scene);
		state.properties().set("target_pose", pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);

		rviz_marker_tools::appendFrame(trajectory.markers(), pose, 0.1, "pose frame");

		spawn(std::move(state), std::move(trajectory));
	}
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
