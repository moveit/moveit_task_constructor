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

/* Authors: Luca Lach, Robert Haschke */

#include <moveit/task_constructor/merge.h>

#include <boost/range/adaptor/transformed.hpp>
#include <boost/algorithm/string/join.hpp>

namespace {
std::vector<const moveit::core::JointModel*>
findDuplicates(const std::vector<const moveit::core::JointModelGroup*>& groups,
               std::vector<const moveit::core::JointModel*> joints) {
	std::vector<const moveit::core::JointModel*> duplicates;
	for (const moveit::core::JointModelGroup* jmg : groups) {
		for (const moveit::core::JointModel* jm : jmg->getJointModels()) {
			auto it = std::find(joints.begin(), joints.end(), jm);
			if (it == joints.end()) {  // jm not found anymore -> duplicate
				if (jm->getType() == moveit::core::JointModel::FIXED || jm->getMimic())
					continue;  // fixed joints and mimic joints are OK
				if (std::find(duplicates.begin(), duplicates.end(), jm) == duplicates.end())
					duplicates.push_back(jm);  // add to duplicates only once
			} else
				joints.erase(it);  // remove from list as processed
		}
	}
	return duplicates;
}
}  // namespace

namespace moveit {
namespace task_constructor {

moveit::core::JointModelGroup* merge(const std::vector<const moveit::core::JointModelGroup*>& groups) {
	if (groups.size() <= 1)
		throw std::runtime_error("Expected multiple groups");

	const moveit::core::RobotModel* const robot_model = &groups[0]->getParentModel();

	std::set<const moveit::core::JointModel*> jset;
	std::string merged_group_name;
	size_t sum_joints = 0;
	for (const moveit::core::JointModelGroup* jmg : groups) {
		// sanity check: all groups must share the same robot model
		if (&jmg->getParentModel() != robot_model)
			throw std::runtime_error("groups refer to different robot models");

		const auto& joints = jmg->getJointModels();
		jset.insert(joints.cbegin(), joints.cend());
		sum_joints += joints.size();
		if (!merged_group_name.empty())
			merged_group_name.append({ '+' });
		merged_group_name.append(jmg->getName());
	}

	std::vector<const moveit::core::JointModel*> joints(jset.cbegin(), jset.cend());
	if (joints.size() != sum_joints) {  // overlapping joint groups: analyse in more detail
		auto duplicates = findDuplicates(groups, joints);
		if (!duplicates.empty()) {
			// NOLINTNEXTLINE(readability-identifier-naming): getJointName is a function (variable)
			auto getJointName = boost::adaptors::transformed([](auto&& jm) { return jm->getName(); });
			std::string message("overlapping joints: " + boost::algorithm::join(duplicates | getJointName, ", "));
			throw std::runtime_error(message);
		}
	}

	// JointModelGroup expects a srdf group,
	// but this model is not constructed from an srdf, so we have to provide a dummy
	static srdf::Model::Group dummy_srdf;
	return new moveit::core::JointModelGroup(merged_group_name, dummy_srdf, joints, robot_model);
}

robot_trajectory::RobotTrajectoryPtr
merge(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
      const moveit::core::RobotState& base_state, moveit::core::JointModelGroup*& merged_group,
      const trajectory_processing::TimeParameterization& time_parameterization) {
	if (sub_trajectories.size() <= 1)
		throw std::runtime_error("Expected multiple sub solutions");

	if (!merged_group) {  // create (and return) a merged group if not yet done
		std::vector<const moveit::core::JointModelGroup*> groups;
		groups.reserve(sub_trajectories.size());
		for (const auto& sub : sub_trajectories)
			groups.push_back(sub->getGroup());
		merged_group = merge(groups);
	}
	const std::vector<const moveit::core::JointModel*>* merged_joints = &merged_group->getJointModels();

	// sanity checks: all sub solutions must share the same robot model and use disjoint joint sets
	const moveit::core::RobotModelConstPtr& robot_model = base_state.getRobotModel();
	unsigned int max_num_vars = 0;  // maximum number of joint variables across sub groups
	for (const robot_trajectory::RobotTrajectoryConstPtr& sub : sub_trajectories) {
		if (sub->getRobotModel() != robot_model)
			throw std::runtime_error("subsolutions refer to multiple robot models");

		const moveit::core::JointModelGroup* jmg = sub->getGroup();
		const auto& joints = jmg->getJointModels();
		// validate that the joint model is known
		for (const moveit::core::JointModel* jm : joints) {
			if (std::find(merged_joints->cbegin(), merged_joints->cend(), jm) == merged_joints->cend())
				throw std::runtime_error("subsolutions refers to unknown joint: " + jm->getName());
		}
		max_num_vars = std::max(max_num_vars, jmg->getVariableCount());
	}

	// do the actual trajectory merging
	auto merged_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, merged_group);
	std::vector<double> values;
	values.reserve(max_num_vars);

	auto merged_state = std::make_shared<moveit::core::RobotState>(base_state);
	while (true) {
		bool finished = true;
		size_t index = merged_traj->getWayPointCount();

		for (const robot_trajectory::RobotTrajectoryConstPtr& sub : sub_trajectories) {
			if (index >= sub->getWayPointCount())
				continue;  // no more waypoints in this sub solution

			finished = false;  // there was a waypoint, continue while loop
			const moveit::core::RobotState& sub_state = sub->getWayPoint(index);
			sub_state.copyJointGroupPositions(sub->getGroup(), values);
			merged_state->setJointGroupPositions(sub->getGroup(), values);
		}
		if (finished)
			break;

		merged_state->update();
		// add waypoint without timing
		merged_traj->addSuffixWayPoint(merged_state, 0.0);
		// create new RobotState for next waypoint
		merged_state = std::make_shared<moveit::core::RobotState>(*merged_state);
	}

	// add timing
	time_parameterization.computeTimeStamps(*merged_traj, 1.0, 1.0);
	return merged_traj;
}
}  // namespace task_constructor
}  // namespace moveit
