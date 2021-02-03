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
#include <rclcpp/rclcpp.hpp>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_demo");

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi,
                 const moveit_msgs::msg::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::msg::CollisionObject createTable(const rclcpp::Node::SharedPtr& node) {
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::msg::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(node, "table_name", table_name);
	errors += !rosparam_shortcuts::get(node, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(node, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(node, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(errors);

	moveit_msgs::msg::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
	object.primitives[0].dimensions = { table_dimensions.at(0), table_dimensions.at(1), table_dimensions.at(2) };
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::msg::CollisionObject createObject(const rclcpp::Node::SharedPtr& node) {
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::msg::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(node, "object_name", object_name);
	error += !rosparam_shortcuts::get(node, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(node, "object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(node, "object_pose", pose);
	rosparam_shortcuts::shutdownIfError(error);

	moveit_msgs::msg::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = { object_dimensions.at(0), object_dimensions.at(1) };
	pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}

int main(int argc, char** argv) {
	RCLCPP_INFO(LOGGER, "Init moveit_task_constructor_demo");
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("moveit_task_constructor_demo");
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	// Add table and object to planning scene
	rclcpp::sleep_for(std::chrono::microseconds(100));  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	if (bool spawn_table = true; rosparam_shortcuts::get(node, "spawn_table", spawn_table) && spawn_table)
		spawnObject(psi, createTable(node));
	spawnObject(psi, createObject(node));

	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", node);
	pick_place_task.loadParameters();
	pick_place_task.init();
	if (pick_place_task.plan()) {
		RCLCPP_INFO(LOGGER, "Planning succeeded");
		if (bool execute = false; rosparam_shortcuts::get(node, "execute", execute) && execute) {
			pick_place_task.execute();
			RCLCPP_INFO(LOGGER, "Execution complete");
		} else {
			RCLCPP_INFO(LOGGER, "Execution disabled");
		}
	} else {
		RCLCPP_INFO(LOGGER, "Planning failed");
	}

	// Keep introspection alive
	spinning_thread.join();
	return 0;
}
