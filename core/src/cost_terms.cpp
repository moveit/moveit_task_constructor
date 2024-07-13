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

#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>

#include <fmt/core.h>
#include <utility>

namespace moveit {
namespace task_constructor {

double CostTerm::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	return s.cost();
}

double CostTerm::operator()(const SolutionSequence& s, std::string& /*comment*/) const {
	return s.cost();
}

double CostTerm::operator()(const WrappedSolution& s, std::string& /*comment*/) const {
	return s.cost();
}

double TrajectoryCostTerm::operator()(const SolutionSequence& s, std::string& comment) const {
	double cost{ 0.0 };
	std::string subcomment;
	for (auto& solution : s.solutions()) {
		cost += solution->computeCost((*this), subcomment);
		if (!subcomment.empty()) {
			if (!comment.empty())
				comment.append(", ");
			comment.append(subcomment);
			subcomment.clear();
		}
	}

	return cost;
}

double TrajectoryCostTerm::operator()(const WrappedSolution& s, std::string& comment) const {
	return s.wrapped()->computeCost(*this, comment);
}

LambdaCostTerm::LambdaCostTerm(const SubTrajectorySignature& term)
  : term_{ [term](const SolutionBase& s, std::string& c) { return term(static_cast<const SubTrajectory&>(s), c); } } {}

LambdaCostTerm::LambdaCostTerm(const SubTrajectoryShortSignature& term)
  : term_{ [term](const SolutionBase& s, std::string& /*c*/) { return term(static_cast<const SubTrajectory&>(s)); } } {}

double LambdaCostTerm::operator()(const SubTrajectory& s, std::string& comment) const {
	assert(bool{ term_ });
	return term_(s, comment);
}

namespace cost {

double Constant::operator()(const SubTrajectory& /*s*/, std::string& /*comment*/) const {
	return cost;
}

double Constant::operator()(const SolutionSequence& /*s*/, std::string& /*comment*/) const {
	return cost;
}

double Constant::operator()(const WrappedSolution& /*s*/, std::string& /*comment*/) const {
	return cost;
}

PathLength::PathLength(std::vector<std::string> joints) {
	for (auto& j : joints)
		this->joints.emplace(std::move(j), 1.0);
}

double PathLength::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	const auto& traj = s.trajectory();

	if (traj == nullptr || traj->getWayPointCount() == 0)
		return 0.0;

	std::map<const moveit::core::JointModel*, double> weights;
	const auto& first_waypoint = traj->getWayPoint(0);
	for (auto& joint_weight : joints) {
		const moveit::core::JointModel* jm = first_waypoint.getJointModel(joint_weight.first);
		if (jm)
			weights.emplace(jm, joint_weight.second);
	}

	double path_length{ 0.0 };
	for (size_t i = 1; i < traj->getWayPointCount(); ++i) {
		auto& last = traj->getWayPoint(i - 1);
		auto& curr = traj->getWayPoint(i);
		if (joints.empty()) {
			path_length += last.distance(curr);
		} else {
			for (const auto& item : weights) {
				path_length += item.second * last.distance(curr, item.first);
			}
		}
	}
	return path_length;
}

DistanceToReference::DistanceToReference(const moveit_msgs::msg::RobotState& ref, Mode m,
                                         std::map<std::string, double> w)
  : reference(ref), weights(std::move(w)), mode(m) {}

DistanceToReference::DistanceToReference(const std::map<std::string, double>& ref, Mode m,
                                         std::map<std::string, double> w)
  : weights(std::move(w)), mode(m) {
	reference.joint_state.name.reserve(ref.size());
	reference.joint_state.position.reserve(ref.size());

	for (auto& item : ref) {
		reference.joint_state.name.push_back(item.first);
		reference.joint_state.position.push_back(item.second);
	}
	reference.is_diff = true;
}

double DistanceToReference::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	const auto& state = (mode == Mode::END_INTERFACE) ? s.end() : s.start();
	const auto& traj = s.trajectory();

	moveit::core::RobotState ref_state = state->scene()->getCurrentState();
	moveit::core::robotStateMsgToRobotState(reference, ref_state, false);

	std::map<const moveit::core::JointModel*, double> w;
	for (auto& item : weights) {
		const moveit::core::JointModel* jm = ref_state.getJointModel(item.first);
		if (jm)
			w.emplace(jm, item.second);
	}

	auto distance = [this, &ref_state, &w](const moveit::core::RobotState& state) {
		if (weights.empty()) {
			return ref_state.distance(state);
		} else {
			double accumulated = 0.0;
			for (const auto& item : w)
				accumulated += item.second * ref_state.distance(state, item.first);
			return accumulated;
		}
	};

	if (mode == Mode::START_INTERFACE || mode == Mode::END_INTERFACE || (mode == Mode::AUTO && (traj == nullptr))) {
		return distance(state->scene()->getCurrentState());
	} else {
		double accumulated = 0.0;
		for (size_t i = 0; i < traj->getWayPointCount(); ++i)
			accumulated += distance(traj->getWayPoint(i));
		accumulated /= traj->getWayPointCount();
		return accumulated;
	}
}

double TrajectoryDuration::operator()(const SubTrajectory& s, std::string& /*comment*/) const {
	return s.trajectory() ? s.trajectory()->getDuration() : 0.0;
}

LinkMotion::LinkMotion(std::string link) : link_name{ std::move(link) } {}

double LinkMotion::operator()(const SubTrajectory& s, std::string& comment) const {
	const auto& traj{ s.trajectory() };

	if (traj == nullptr || traj->getWayPointCount() == 0)
		return 0.0;

	if (!traj->getWayPoint(0).knowsFrameTransform(link_name)) {
		comment = fmt::format("LinkMotionCost: frame '{}' unknown in trajectory", link_name);
		return std::numeric_limits<double>::infinity();
	}

	double distance{ 0.0 };
	Eigen::Vector3d position{ traj->getWayPoint(0).getFrameTransform(link_name).translation() };
	for (size_t i{ 1 }; i < traj->getWayPointCount(); ++i) {
		const auto& new_position{ traj->getWayPoint(i).getFrameTransform(link_name).translation() };
		distance += (new_position - position).norm();
		position = new_position;
	}
	return distance;
}

Clearance::Clearance(bool with_world, bool cumulative, std::string group_property, Mode mode)
  : with_world{ with_world }
  , cumulative{ cumulative }
  , group_property{ std::move(group_property) }
  , mode{ mode }
  , distance_to_cost{ [](double d) { return 1.0 / (d + 1e-5); } } {}

double Clearance::operator()(const SubTrajectory& s, std::string& comment) const {
	static const std::string PREFIX{ "Clearance: " };

	collision_detection::DistanceRequest request;
	request.type =
	    cumulative ? collision_detection::DistanceRequestType::SINGLE : collision_detection::DistanceRequestType::GLOBAL;

	const auto& state{ (mode == Mode::END_INTERFACE) ? s.end() : s.start() };

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

	auto collision_comment = [=](const auto& distance) {
		return fmt::format(PREFIX + "allegedly valid solution collides between '{}' and '{}'", distance.link_names[0],
		                   distance.link_names[1]);
	};

	double distance{ 0.0 };

	if (mode == Mode::START_INTERFACE || mode == Mode::END_INTERFACE ||
	    (mode == Mode::AUTO && s.trajectory() == nullptr)) {
		auto distance_data{ check_distance(state, state->scene()->getCurrentState()) };
		if (distance_data.distance < 0) {
			comment = collision_comment(distance_data);
			return std::numeric_limits<double>::infinity();
		}
		distance = distance_data.distance;
		if (!cumulative)
			comment = fmt::format(PREFIX + "distance {} between '{}' and '{}'", distance, distance_data.link_names[0],
			                      distance_data.link_names[1]);
		else
			comment = fmt::format(PREFIX + "cumulative distance {}", distance);
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
		comment = fmt::format(PREFIX + "average{} distance: {}", (cumulative ? " cumulative" : ""), distance);
	}

	return distance_to_cost(distance);
}
}  // namespace cost
}  // namespace task_constructor
}  // namespace moveit
