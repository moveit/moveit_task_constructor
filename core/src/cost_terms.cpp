/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Hamburg University
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
 *   * Neither the name of the copyright holders nor the names of its
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

#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stage.h>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/collision_detection/collision_common.h>

namespace moveit {
namespace task_constructor {
namespace cost {

double PathLengthCost(const SubTrajectory& s) {
	return s.trajectory() ? s.trajectory()->getDuration() : 0.0;
}

double ClearanceCost(const SubTrajectory& s) {
	collision_detection::DistanceRequest request;
	request.type = collision_detection::DistanceRequestType::GLOBAL;

	// TODO: possibly parameterize hardcoded property name?
	const std::string group{ "group" };
	auto& state_properties{ s.start()->properties() };
	auto& stage_properties{ s.creator()->properties() };
	request.group_name = state_properties.hasProperty(group) ? state_properties.get<std::string>(group) :
	                                                           stage_properties.get<std::string>(group);

	request.enableGroup(s.start()->scene()->getRobotModel());
	request.acm = &s.start()->scene()->getAllowedCollisionMatrix();

	collision_detection::DistanceResult result;

	s.start()->scene()->getCollisionEnv()->distanceSelf(request, result, s.start()->scene()->getCurrentState());

	if (result.minimum_distance.distance <= 0) {
		// TODO: this should be a comment in the solution
		ROS_ERROR_STREAM_NAMED("ClearanceCost", "allegedly valid solution has an unwanted collide between '"
		                                            << result.minimum_distance.link_names[0] << "' and '"
		                                            << result.minimum_distance.link_names[1] << "'");
		return std::numeric_limits<double>::infinity();
	} else {
		// ROS_INFO_STREAM_NAMED("ClearanceCost", result.minimum_distance.link_names[0] << " has distance " <<
		// result.minimum_distance.distance << " to " << result.minimum_distance.link_names[1]);
		return 1.0 / (result.minimum_distance.distance + 1e-5);
	}
}
}
}
}
