/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MTC
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/generate_pose.h>
#include <moveit/task_constructor/stages/generate_place_pose.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/pick.h>

#include <moveit/task_constructor/stages/grasp_provider_base.h>
#include <moveit/task_constructor/stages/place_provider_base.h>

#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/PlaceLocation.h>

#include <pluginlib/class_loader.h>

#pragma once

namespace moveit {
namespace task_constructor {
namespace tasks {

using GraspProviderPluginLoader = pluginlib::ClassLoader<stages::GraspProviderBase>;
using PlaceProviderPluginLoader = pluginlib::ClassLoader<stages::PlaceProviderBase>;
using namespace moveit::task_constructor;

class PickPlaceTask
{
public:
  struct Parameters
  {
    short task_type_;

    // planning group properties
    std::string arm_group_name_;
    std::string eef_name_;
    std::string hand_group_name_;
    std::string hand_frame_;

    // object + surface
    std::vector<std::string> support_surfaces_;
    std::string object_name_;

    // Plugins
    std::string grasp_provider_plugin_name_;
    std::string place_provider_plugin_name_;

    // Pick metrics
    std::vector<moveit_msgs::Grasp> grasps_;
    Eigen::Isometry3d grasp_frame_transform_;

    // Place metrics
    std::vector<moveit_msgs::PlaceLocation> place_locations_;
  };

	PickPlaceTask(const std::string& task_name);
	~PickPlaceTask() = default;

  bool init(const Parameters& parameters);

	bool plan();
  void getSolutionMsg(moveit_task_constructor_msgs::Solution& solution);

private:
	TaskPtr task_;
  const std::string task_name_;
  Stage* current_state_stage_;
	Stage* attach_object_stage_;
  
  std::unique_ptr<GraspProviderPluginLoader> grasp_provider_class_loader_;
  std::unique_ptr<PlaceProviderPluginLoader> place_provider_class_loader_;
};

}  // namespace tasks
}  // namespace task_constructor
}  // namespace moveit
