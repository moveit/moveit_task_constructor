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

#include <boost/format.hpp>

namespace moveit {
namespace task_constructor {
namespace cost {

double PathLengthCost(const SubTrajectory& s) {
	const auto& traj = s.trajectory();

	if (traj == nullptr)
		return 0.0;

	double path_length{ 0.0 };
	for (size_t i = 1; i < traj->getWayPointCount(); ++i)
		path_length += traj->getWayPoint(i - 1).distance(traj->getWayPoint(i));
	return path_length;
}

double TrajectoryDurationCost(const SubTrajectory& s) {
	return s.trajectory() ? s.trajectory()->getDuration() : 0.0;
}

double ClearanceCost::operator()(const SubTrajectory& s, std::string& comment) {
	collision_detection::DistanceRequest request;
	request.type =
	    cumulative ? collision_detection::DistanceRequestType::SINGLE : collision_detection::DistanceRequestType::GLOBAL;

	const auto& state = (interface == Interface::START) ? s.start() : s.end();

	// prefer interface state property over stage property to find group_name
	// TODO: This pattern is general enough to justify its own interface (in the properties?).
	auto& state_properties{ state->properties() };
	auto& stage_properties{ s.creator()->properties() };
	request.group_name = state_properties.hasProperty(group_property) ?
	                         state_properties.get<std::string>(group_property) :
	                         stage_properties.get<std::string>(group_property);

	// look at all forbidden collisions involving group_name
	request.enableGroup(state->scene()->getRobotModel());
	request.acm = &state->scene()->getAllowedCollisionMatrix();

	collision_detection::DistanceResult result;

	state->scene()->getCollisionEnv()->distanceSelf(request, result, state->scene()->getCurrentState());

	double distance{ 0.0 };
	if (cumulative) {
		for (const auto& distance_of_pair : result.distances) {
			assert(distance_of_pair.second.size() == 1);
			distance += distance_of_pair.second[0].distance;
		}
	} else {
		distance = result.minimum_distance.distance;
	}

	const auto& links = result.minimum_distance.link_names;

	if (result.minimum_distance.distance <= 0) {
		boost::format desc("ClearCost: allegedly valid solution has an unwanted collide between '%1%' and '%2%'");
		desc % links[0] % links[1];
		comment = desc.str();
		return std::numeric_limits<double>::infinity();
	} else {
		if (cumulative) {
			comment = "ClearCost: cumulative distance " + std::to_string(distance);
		} else {
			boost::format desc("ClearCost: distance %1% between '%2%' and '%3%'");
			desc % result.minimum_distance.distance % links[0] % links[1];
			comment = desc.str();
		}
		return 1.0 / (distance + 1e-5);
	}
}
}
}
}
