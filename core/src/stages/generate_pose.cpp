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

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/marker_tools.h>
#include <moveit/planning_scene/planning_scene.h>
#include <rviz_marker_tools/marker_creation.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>

#include <chrono>

namespace {
// TODO(henningkayser): support user-defined random number engines
std::random_device rd_;
std::mt19937 gen_(rd_());
}

namespace moveit {
namespace task_constructor {
namespace stages {

GeneratePose::GeneratePose(const std::string& name) : MonitoringGenerator(name) {
	setCostTerm(std::make_unique<cost::Constant>(0.0));

	auto& p = properties();
	p.declare<geometry_msgs::PoseStamped>("pose", "target pose to pass on in spawned states");
	p.declare<size_t>("max_solutions", 20,
	                  "limit of the number of spawned solution in case randomized sampling is enabled");
	p.property("timeout").setDefaultValue(1.0 /* seconds */);
}

template <>
GeneratePose::PoseDimensionSampler GeneratePose::getPoseDimensionSampler<std::normal_distribution>(double stddev) {
	return [ stddev, &gen = gen_ ](double mean) {
		static std::normal_distribution<double> dist(mean, stddev);
		return dist(gen);
	};
}

template <>
GeneratePose::PoseDimensionSampler GeneratePose::getPoseDimensionSampler<std::uniform_real_distribution>(double range) {
	return [ range, &gen = gen_ ](double mean) {
		static std::uniform_real_distribution<double> dist(mean - 0.5 * range, mean + 0.5 * range);
		return dist(gen);
	};
}

void GeneratePose::reset() {
	upstream_solutions_.clear();
	MonitoringGenerator::reset();
}

void GeneratePose::onNewSolution(const SolutionBase& s) {
	// It's safe to store a pointer to this solution, as the generating stage stores it
	upstream_solutions_.push(&s);
}

bool GeneratePose::canCompute() const {
	return !upstream_solutions_.empty();
}

void GeneratePose::compute() {
	if (upstream_solutions_.empty())
		return;

	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();
	geometry_msgs::PoseStamped target_pose = properties().get<geometry_msgs::PoseStamped>("pose");
	if (target_pose.header.frame_id.empty())
		target_pose.header.frame_id = scene->getPlanningFrame();
	else if (!scene->knowsFrameTransform(target_pose.header.frame_id)) {
		ROS_WARN_NAMED("GeneratePose", "Unknown frame: '%s'", target_pose.header.frame_id.c_str());
		return;
	}

	const auto& spawn_target_pose = [&](const geometry_msgs::PoseStamped& target_pose) {
		InterfaceState state(scene);
		state.properties().set("target_pose", target_pose);

		SubTrajectory trajectory;
		trajectory.setCost(0.0);

		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose, 0.1, "pose frame");

		spawn(std::move(state), std::move(trajectory));
	};

	spawn_target_pose(target_pose);

	// If no dimension samplers were defined, we're done here
	if (pose_dimension_samplers_.empty())
		return;

	// Use target pose as seed pose
	geometry_msgs::PoseStamped seed_pose = target_pose;
	tf2::Quaternion seed_q, rand_q;
	tf2::fromMsg(seed_pose.pose.orientation, seed_q);
	double elapsed_time = 0.0;
	const auto start_time = std::chrono::steady_clock::now();
	size_t spawned_solutions = 0;
	while (elapsed_time < timeout() && spawned_solutions++ < properties().get<size_t>("max_solutions") - 1) {
		// Randomize pose using specified dimension samplers.
		// RPY dimensions are using 0.0 as seed, the randomized rotation is multiplied to the target orientation.
		double rand_roll = 0.0, rand_pitch = 0.0, rand_yaw = 0.0;
		for (const auto& pose_dim_sampler : pose_dimension_samplers_) {
			switch (pose_dim_sampler.first) {
				case X:
					target_pose.pose.position.x = pose_dim_sampler.second(seed_pose.pose.position.x);
					break;
				case Y:
					target_pose.pose.position.y = pose_dim_sampler.second(seed_pose.pose.position.y);
					break;
				case Z:
					target_pose.pose.position.z = pose_dim_sampler.second(seed_pose.pose.position.z);
					break;
				case ROLL:
					rand_roll = pose_dim_sampler.second(rand_roll);
					break;
				case PITCH:
					rand_pitch = pose_dim_sampler.second(rand_pitch);
					break;
				case YAW:
					rand_yaw = pose_dim_sampler.second(rand_yaw);
			}
		}
		rand_q.setRPY(rand_roll, rand_pitch, rand_yaw);
		rand_q = seed_q * rand_q;
		target_pose.pose.orientation = tf2::toMsg(rand_q);

		// Spawn sampled pose
		spawn_target_pose(target_pose);

		elapsed_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
	}
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
