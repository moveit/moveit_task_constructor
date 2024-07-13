/*********************************************************************
 * Software License Agreement (BSD License)
 *
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

/* Author: Robert Haschke */

#include <moveit/visualization_tools/display_solution.h>
#include <moveit/visualization_tools/marker_visualization.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <rclcpp/logging.hpp>
#include <fmt/core.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.display_solution");

namespace moveit_rviz_plugin {

std::pair<size_t, size_t> DisplaySolution::indexPair(size_t index) const {
	size_t part = 0;
	for (const auto& d : data_) {
		if (index < d.trajectory_->getWayPointCount())
			break;
		index -= d.trajectory_->getWayPointCount();
		++part;
	}
	assert(part < data_.size());
	assert(index < data_[part].trajectory_->getWayPointCount());
	return std::make_pair(part, index);
}

DisplaySolution::DisplaySolution(const DisplaySolution& master, uint32_t sub)
  : start_scene_(sub == 0 ? master.start_scene_ : master.data_[sub - 1].scene_), data_({ master.data_[sub] }) {
	steps_ = data_.front().trajectory_->getWayPointCount();
}

float DisplaySolution::getWayPointDurationFromPrevious(const IndexPair& idx_pair) const {
	return data_[idx_pair.first].trajectory_->getWayPointDurationFromPrevious(idx_pair.second);
}

const moveit::core::RobotStatePtr& DisplaySolution::getWayPointPtr(const IndexPair& idx_pair) const {
	return data_[idx_pair.first].trajectory_->getWayPointPtr(idx_pair.second);
}

const planning_scene::PlanningSceneConstPtr& DisplaySolution::scene(const IndexPair& idx_pair) const {
	// start scene is parent of end scene
	return data_[idx_pair.first].scene_->getParent();
}

const std::string& DisplaySolution::comment(const IndexPair& idx_pair) const {
	return data_[idx_pair.first].comment_;
}

uint32_t DisplaySolution::creatorId(const DisplaySolution::IndexPair& idx_pair) const {
	return data_[idx_pair.first].creator_id_;
}

const MarkerVisualizationPtr DisplaySolution::markers(const DisplaySolution::IndexPair& idx_pair) const {
	return data_[idx_pair.first].markers_;
}

void DisplaySolution::setFromMessage(const planning_scene::PlanningScenePtr& start_scene,
                                     const moveit_task_constructor_msgs::msg::Solution& msg) {
	if (msg.start_scene.robot_model_name != start_scene->getRobotModel()->getName())
		throw std::invalid_argument(fmt::format("Solution for model '{}' but model '{}' was expected",
		                                        msg.start_scene.robot_model_name,
		                                        start_scene->getRobotModel()->getName()));

	// initialize parent scene from solution's start scene
	start_scene->setPlanningSceneMsg(msg.start_scene);
	start_scene_ = start_scene;
	planning_scene::PlanningScenePtr ref_scene = start_scene_->diff();

	data_.resize(msg.sub_trajectory.size());

	steps_ = 0;
	size_t i = 0;
	for (const auto& sub : msg.sub_trajectory) {
		data_[i].trajectory_.reset(new robot_trajectory::RobotTrajectory(ref_scene->getRobotModel(), nullptr));
		data_[i].trajectory_->setRobotTrajectoryMsg(ref_scene->getCurrentState(), sub.trajectory);
		data_[i].joints_ = sub.trajectory.joint_trajectory.joint_names;
		data_[i].joints_.insert(data_[i].joints_.end(), sub.trajectory.multi_dof_joint_trajectory.joint_names.begin(),
		                        sub.trajectory.multi_dof_joint_trajectory.joint_names.end());
		data_[i].comment_ = sub.info.comment;
		data_[i].creator_id_ = sub.info.stage_id;
		steps_ += data_[i].trajectory_->getWayPointCount();

		ref_scene->setPlanningSceneDiffMsg(sub.scene_diff);
		data_[i].scene_ = ref_scene;

		// create new reference scene for next iteration
		ref_scene = ref_scene->diff();

		if (!sub.info.markers.empty())
			data_[i].markers_.reset(new MarkerVisualization(sub.info.markers, *ref_scene));
		else
			data_[i].markers_.reset();
		++i;
	}
}

void DisplaySolution::fillMessage(moveit_task_constructor_msgs::msg::Solution& msg) const {
	start_scene_->getPlanningSceneMsg(msg.start_scene);
	msg.sub_trajectory.resize(data_.size());
	auto traj_it = msg.sub_trajectory.begin();
	for (const auto& sub : data_) {
		sub.scene_->getPlanningSceneDiffMsg(traj_it->scene_diff);
		sub.trajectory_->getRobotTrajectoryMsg(traj_it->trajectory, sub.joints_);
		++traj_it;
	}
}

}  // namespace moveit_rviz_plugin
