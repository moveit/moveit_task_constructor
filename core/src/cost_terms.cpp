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

double PathLength(const SubTrajectory& s) {
	const auto& traj = s.trajectory();

	if (traj == nullptr)
		return 0.0;

	double path_length{ 0.0 };
	for (size_t i = 1; i < traj->getWayPointCount(); ++i)
		path_length += traj->getWayPoint(i - 1).distance(traj->getWayPoint(i));
	return path_length;
}

double TrajectoryDuration(const SubTrajectory& s) {
	return s.trajectory() ? s.trajectory()->getDuration() : 0.0;
}

double LinkMotion::operator()(const SubTrajectory& s, std::string& comment) {
	const auto& traj{ s.trajectory() };

	if (traj == nullptr || traj->getWayPointCount() == 0)
		return 0.0;

	if (!traj->getWayPoint(0).knowsFrameTransform(link_name)) {
		boost::format desc("LinkMotionCost: frame '%1%' unknown in trajectory");
		desc % link_name;
		comment = desc.str();
		return std::numeric_limits<double>::infinity();
	}

	double distance{ 0.0 };
	Eigen::Translation3d position{ traj->getWayPoint(0).getFrameTransform(link_name).translation() };
	for (size_t i{ 1 }; i < traj->getWayPointCount(); ++i) {
		Eigen::Translation3d new_position{ traj->getWayPoint(i).getFrameTransform(link_name).translation() };
		distance += (new_position.vector() - position.vector()).norm();
		position = new_position;
	}
	return distance;
}

double Clearance::operator()(const SubTrajectory& s, std::string& comment) {
	const std::string PREFIX{ "Clearance: " };

	collision_detection::DistanceRequest request;
	request.type =
	    cumulative ? collision_detection::DistanceRequestType::SINGLE : collision_detection::DistanceRequestType::GLOBAL;

	const auto& state{ (interface == Interface::END) ? s.end() : s.start() };

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

	// compute relevant distance data for state & robot
	auto check_distance{ [=](const InterfaceState* state, const moveit::core::RobotState& robot) {
		collision_detection::DistanceResult result;
		if (with_world)
			state->scene()->getCollisionEnv()->distanceRobot(request, result, robot);
		else
			state->scene()->getCollisionEnv()->distanceSelf(request, result, robot);

		if (result.minimum_distance.distance <= 0) {
			return result.minimum_distance;
		}

		if (cumulative) {
			double distance{ 0.0 };
			for (const auto& distance_of_pair : result.distances) {
				assert(distance_of_pair.second.size() == 1);
				distance += distance_of_pair.second[0].distance;
			}
			result.minimum_distance.distance = distance;
		}

		return result.minimum_distance;
	} };

	auto collision_comment{ [=](const auto& distance) {
		boost::format desc{ PREFIX + "allegedly valid solution collides between '%1%' and '%2%'" };
		desc % distance.link_names[0] % distance.link_names[1];
		return desc.str();
	} };

	double distance{ 0.0 };

	if (interface == Interface::START || interface == Interface::END ||
	    (interface == Interface::NONE && s.trajectory() == nullptr)) {
		auto distance_data{ check_distance(state, state->scene()->getCurrentState()) };
		if (distance_data.distance < 0) {
			comment = collision_comment(distance_data);
			return std::numeric_limits<double>::infinity();
		}
		distance = distance_data.distance;
		if (!cumulative) {
			boost::format desc{ PREFIX + "distance %1% between '%2%' and '%3%'" };
			desc % distance % distance_data.link_names[0] % distance_data.link_names[1];
			comment = desc.str();
		} else {
			comment = PREFIX + "cumulative distance " + std::to_string(distance);
		}
	} else {  // check trajectory
		for (size_t i = 0; i < s.trajectory()->getWayPointCount(); ++i) {
			auto distance_data = check_distance(state, s.trajectory()->getWayPoint(i));
			if (distance_data.distance < 0) {
				comment = collision_comment(distance_data);
				return std::numeric_limits<double>::infinity();
			}
			distance += distance_data.distance;
		}
		distance /= s.trajectory()->getWayPointCount();

		boost::format desc(PREFIX + "average%1% distance: %2%");
		desc % (cumulative ? " cumulative" : "") % distance;
		comment = desc.str();
	}

	return 1.0 / (distance + 1e-5);
}
}
}
}
