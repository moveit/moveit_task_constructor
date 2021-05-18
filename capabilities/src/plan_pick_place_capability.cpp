/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Henning Kayser */

#include "plan_pick_place_capability.h"

#include <moveit/move_group/capability_names.h>
#include <moveit/robot_state/conversions.h>


namespace move_group {

PlanPickPlaceCapability::PlanPickPlaceCapability() : MoveGroupCapability("PlanPickPlace") {}

void PlanPickPlaceCapability::initialize() {
	// Configure the action server
	as_.reset(new actionlib::SimpleActionServer<moveit_task_constructor_msgs::PlanPickPlaceAction>(
	    root_node_handle_, "plan_pick_place",
	    std::bind(&PlanPickPlaceCapability::goalCallback, this, std::placeholders::_1), false));
	as_->registerPreemptCallback(std::bind(&PlanPickPlaceCapability::preemptCallback, this));
  as_->start();
  pick_place_task_ = std::make_unique<PickPlaceTask>("pick_place_task");
}

void PlanPickPlaceCapability::goalCallback(
    const moveit_task_constructor_msgs::PlanPickPlaceGoalConstPtr& goal) {
	moveit_task_constructor_msgs::PlanPickPlaceResult result;

  // TODO: fill parameters
  PickPlaceTask::Parameters parameters;
  parameters.task_type_ = goal->task_type;
  parameters.arm_group_name_ = goal->arm_group_name;
  parameters.hand_group_name_ = goal->hand_group_name;
  parameters.eef_name_ = goal->eef_name;
  parameters.hand_frame_ = goal->hand_frame;
  parameters.object_name_ = goal->object_id;
  parameters.support_surfaces_ = goal->support_surfaces;
  parameters.grasp_pose_ = goal->grasp.grasp_pose;
  parameters.grasp_provider_plugin_name_ = goal->grasp_provider_plugin_name;
  tf::poseMsgToEigen(goal->grasp_frame_transform, parameters.grasp_frame_transform_);
  parameters.hand_open_pose_ = "open";
  parameters.hand_close_pose_ = "close";
  parameters.approach_object_direction_ = goal->grasp.pre_grasp_approach.direction;
  parameters.approach_object_min_dist_ = goal->grasp.pre_grasp_approach.min_distance;
  parameters.approach_object_max_dist_ = goal->grasp.pre_grasp_approach.desired_distance;
  parameters.lift_object_direction_ = goal->grasp.post_grasp_retreat.direction;
  parameters.lift_object_min_dist_ = goal->grasp.post_grasp_retreat.min_distance;
  parameters.lift_object_max_dist_ = goal->grasp.post_grasp_retreat.desired_distance;

  parameters.place_provider_plugin_name_ = goal->place_provider_plugin_name;
  parameters.place_pose_ = goal->place_location.place_pose;
  parameters.place_object_direction_ = goal->place_location.pre_place_approach.direction;
  parameters.place_object_min_dist_ = goal->place_location.pre_place_approach.min_distance;
  parameters.place_object_max_dist_ = goal->place_location.pre_place_approach.desired_distance;
  parameters.retract_direction_ = goal->place_location.post_place_retreat.direction;
  parameters.retract_min_dist_ = goal->place_location.post_place_retreat.min_distance;
  parameters.retract_max_dist_ = goal->place_location.post_place_retreat.desired_distance;

  // Initialize task and plan
  if (pick_place_task_->init(parameters)){
    // Compute plan
    result.success = pick_place_task_->plan();
    if (result.success) {
      pick_place_task_->getSolutionMsg(result.solution);
    }
  } else {
    result.success = false;
  }
  // Retrieve and return result
    as_->setSucceeded(result);
}

void PlanPickPlaceCapability::preemptCallback() {
  // TODO(henningkayser): abort planning
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::PlanPickPlaceCapability, move_group::MoveGroupCapability)
