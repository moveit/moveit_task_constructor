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

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

void spawnTable()
{
  ros::Duration(1.0).sleep();
  ros::NodeHandle pnh("~");
  std::string table_name = pnh.param<std::string>("table_name", "table");
  std::string surface_frame = pnh.param<std::string>("table_surface_frame", "world");
  double height = pnh.param<double>("table_height", 0.3);
  double width = pnh.param<double>("table_width", 0.5);
  double length = pnh.param<double>("table_length", 0.5);
  double position_x = pnh.param<double>("table_pos_x", 0.5);
  double position_y = pnh.param<double>("table_pos_y", 0.0);

  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject object;
  object.id = table_name = table_name;
  object.header.frame_id = surface_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  object.primitives[0].dimensions = { length, width, height };
  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = position_x;
  object.primitive_poses[0].position.y = position_y;
  object.primitive_poses[0].position.z = 0.5 * height;
  object.primitive_poses[0].orientation.w = 1.0;
  psi.applyCollisionObject(object);
  ros::Duration(1.0).sleep();
}

void spawnObject()
{
  ros::Duration(1.0).sleep();
  ros::NodeHandle pnh("~");
  std::string object_name = pnh.param<std::string>("object_name", "object");
  std::string surface_frame = pnh.param<std::string>("object_surface_frame", "world");
  double height = pnh.param<double>("object_height", 0.2);
  double radius = pnh.param<double>("object_radius", 0.03);
  double position_x = pnh.param<double>("object_pos_x", 0.0);
  double position_y = pnh.param<double>("object_pos_y", 0.0);
  double table_height = pnh.param<double>("table_height", 0.3);
  double place_surface_offset = pnh.param<double>("place_surface_offset", 0.0001);

  moveit::planning_interface::PlanningSceneInterface psi;
  moveit_msgs::CollisionObject object;
  object.id = object_name = object_name;
  object.header.frame_id = surface_frame;
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { height, radius };
  object.primitive_poses.resize(1);
  object.primitive_poses[0].position.x = position_x;
  object.primitive_poses[0].position.y = position_y;
  object.primitive_poses[0].position.z = 0.5 * (height + table_height) + place_surface_offset;
  object.primitive_poses[0].orientation.w = 1.0;
  psi.applyCollisionObject(object);
  ros::Duration(1.0).sleep();
}

int main(int argc, char** argv)
{
  ROS_INFO("Init moveit_task_constructor_demo");
  ros::init(argc, argv, "moveit_task_constructor_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Add table and object to planning scene
  spawnTable();
  spawnObject();

  // Construct and run pick/place task
  moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
  pick_place_task.init();
  if (pick_place_task.plan())
    pick_place_task.execute();

  // Keep introspection alive
  ros::waitForShutdown();
  return 0;
}
