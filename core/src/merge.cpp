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
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace moveit {
namespace task_constructor {

moveit::core::JointModelGroup* merge(const std::vector<const moveit::core::JointModelGroup*>& groups) {
	if (groups.size() <= 1)
		throw std::runtime_error("Expected multiple groups");

	const moveit::core::RobotModel* const robot_model = &groups[0]->getParentModel();

	std::set<const moveit::core::JointModel*> jset;
	std::vector<std::string> names;
	for (const moveit::core::JointModelGroup* jmg : groups) {
		// sanity check: all groups must share the same robot model
		if (&jmg->getParentModel() != robot_model)
			throw std::runtime_error("groups refer to different robot models");

		const auto& joints = jmg->getJointModels();
		jset.insert(joints.cbegin(), joints.cend());
		names.push_back(jmg->getName());
	}

	std::vector<const moveit::core::JointModel*> joints(jset.cbegin(), jset.cend());
	// attention: do not use getConfig()
	return new moveit::core::JointModelGroup("", srdf::Model::Group(), joints, robot_model);
}

bool findDuplicates(const std::vector<const moveit::core::JointModelGroup*>& groups,
                    std::vector<const moveit::core::JointModel*> joints,
                    std::vector<const moveit::core::JointModel*>& duplicates, std::string& names) {
	duplicates.clear();
	names.clear();
	for (const moveit::core::JointModelGroup* jmg : groups) {
		for (const moveit::core::JointModel* jm : jmg->getJointModels()) {
			auto it = std::find(joints.begin(), joints.end(), jm);
			if (it == joints.end()) {  // jm not found anymore -> duplicate
				if (jm->getType() != moveit::core::JointModel::FIXED &&  // fixed joints are OK
				    std::find(duplicates.begin(), duplicates.end(), jm) == duplicates.end()) {
					duplicates.push_back(jm);
					if (!names.empty())
						names.append(", ");
					names.append(jm->getName());
				}
				continue;
			} else
				joints.erase(it);  // remove from list as processed
		}
	}
	return duplicates.size() > 0;
}

robot_trajectory::RobotTrajectoryPtr
merge(const std::vector<robot_trajectory::RobotTrajectoryConstPtr>& sub_trajectories,
      const robot_state::RobotState& base_state, moveit::core::JointModelGroup*& merged_group) {
	if (sub_trajectories.size() <= 1)
		throw std::runtime_error("Expected multiple sub solutions");

	const std::vector<const moveit::core::JointModel*>* merged_joints =
	    merged_group ? &merged_group->getJointModels() : nullptr;
	std::set<const moveit::core::JointModel*> jset;
	std::vector<const moveit::core::JointModelGroup*> groups;
	groups.reserve(sub_trajectories.size());

	// sanity checks: all sub solutions must share the same robot model and use disjoint joint sets
	const moveit::core::RobotModelConstPtr& robot_model = base_state.getRobotModel();
	size_t max_num_joints = 0;  // maximum number of joints in sub groups
	size_t num_joints = 0;  // sum of joints in all sub groups

	for (const robot_trajectory::RobotTrajectoryConstPtr& sub : sub_trajectories) {
		if (sub->getRobotModel() != robot_model)
			throw std::runtime_error("subsolutions refer to multiple robot models");

		const moveit::core::JointModelGroup* jmg = sub->getGroup();
		groups.push_back(jmg);
		const auto& joints = jmg->getJointModels();
		if (merged_joints) {  // validate that the joint model is known in merged_group
			for (const moveit::core::JointModel* jm : joints) {
				if (std::find(merged_joints->cbegin(), merged_joints->cend(), jm) == merged_joints->cend())
					throw std::runtime_error("subsolutions refers to unknown joint: " + jm->getName());
			}
		} else  // accumulate set of joints
			jset.insert(joints.cbegin(), joints.cend());

		max_num_joints = std::max(max_num_joints, joints.size());
		num_joints += joints.size();
	}

	size_t num_merged = merged_joints ? merged_joints->size() : jset.size();
	if (num_merged != num_joints) {
		// overlapping joint groups: analyse in more detail
		std::vector<const moveit::core::JointModel*> joints;
		if (merged_joints)
			joints = *merged_joints;
		else
			joints.insert(joints.end(), jset.cbegin(), jset.cend());

		std::vector<const moveit::core::JointModel*> duplicates;
		std::string names;
		if (findDuplicates(groups, joints, duplicates, names))
			throw std::runtime_error("overlapping joint groups: " + names);
	}

	// create merged_group if necessary
	if (!merged_group) {
		std::vector<const moveit::core::JointModel*> joints(jset.cbegin(), jset.cend());
		merged_group = new moveit::core::JointModelGroup("", srdf::Model::Group(), joints, robot_model.get());
	}

	// do the actual trajectory merging
	auto merged_traj = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, merged_group);
	std::vector<double> values;
	values.reserve(max_num_joints);

	auto merged_state = std::make_shared<robot_state::RobotState>(base_state);
	while (true) {
		bool finished = true;
		size_t index = merged_traj->getWayPointCount();

		for (const robot_trajectory::RobotTrajectoryConstPtr& sub : sub_trajectories) {
			if (index >= sub->getWayPointCount())
				continue;  // no more waypoints in this sub solution

			finished = false;  // there was a waypoint, continue while loop
			const robot_state::RobotState& sub_state = sub->getWayPoint(index);
			sub_state.copyJointGroupPositions(sub->getGroup(), values);
			merged_state->setJointGroupPositions(sub->getGroup(), values);
			merged_state->update();
		}
		if (finished)
			break;

		// add waypoint without timing
		merged_traj->addSuffixWayPoint(merged_state, 0.0);
		// create new RobotState for next waypoint
		merged_state = std::make_shared<robot_state::RobotState>(*merged_state);
	}

	// add timing
	trajectory_processing::IterativeParabolicTimeParameterization timing;
	timing.computeTimeStamps(*merged_traj, 1.0, 1.0);
	return merged_traj;
}
}
}
