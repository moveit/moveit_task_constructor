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
	// configure the action server
	as_.reset(new actionlib::SimpleActionServer<moveit_task_constructor_msgs::PlanPickPlaceAction>(
	    root_node_handle_, "plan_pick_place",
	    std::bind(&PlanPickPlaceCapability::goalCallback, this, std::placeholders::_1), false));
	as_->registerPreemptCallback(std::bind(&PlanPickPlaceCapability::preemptCallback, this));
  as_->start();
  ros::NodeHandle nh("~");
  pick_place_task_ = std::make_unique<PickPlaceTask>("", nh);
}

void PlanPickPlaceCapability::goalCallback(
    const moveit_task_constructor_msgs::PlanPickPlaceGoalConstPtr& goal) {
	moveit_task_constructor_msgs::PlanPickPlaceResult result;

  // initialize task
  // run plan
  // return result
}

void PlanPickPlaceCapability::preemptCallback() {
  // TODO(henningkayser): abort planning
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::PlanPickPlaceCapability, move_group::MoveGroupCapability)
