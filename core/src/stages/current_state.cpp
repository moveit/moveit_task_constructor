/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner, Luca Lach, Robert Haschke */

#include <chrono>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/storage.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <rclcpp/rclcpp.hpp>

namespace moveit {
namespace task_constructor {
namespace stages {

using namespace std::chrono_literals;
constexpr std::chrono::duration<double> DEFAULT_TIMEOUT = 3s;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("CurrentState");

CurrentState::CurrentState(const std::string& name) : Generator(name) {
	auto& p = properties();
	Property& timeout = p.property("timeout");
	timeout.setDescription("max time to wait for get_planning_scene service");
	timeout.setValue(DEFAULT_TIMEOUT.count());
}

void CurrentState::init(const moveit::core::RobotModelConstPtr& robot_model) {
	Generator::init(robot_model);
	robot_model_ = robot_model;
	scene_.reset();
}

bool CurrentState::canCompute() const {
	return !scene_;
}

void CurrentState::compute() {
	scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

	// Add random ID to prevent warnings about multiple publishers within the same node
	auto node = rclcpp::Node::make_shared("current_state_" + std::to_string(reinterpret_cast<std::size_t>(this)));
	auto client = node->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");

	auto timeout = std::chrono::duration<double>(this->timeout());
	if (client->wait_for_service(timeout)) {
		auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();

		req->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
		                             moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
		                             moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
		                             moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_NAMES |
		                             moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
		                             moveit_msgs::msg::PlanningSceneComponents::OCTOMAP |
		                             moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
		                             moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
		                             moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
		                             moveit_msgs::msg::PlanningSceneComponents::OBJECT_COLORS;

		auto res_future = client->async_send_request(req);
		if (rclcpp::spin_until_future_complete(node, res_future) == rclcpp::FutureReturnCode::SUCCESS) {
			auto res = res_future.get();
			scene_->setPlanningSceneMsg(res->scene);
			spawn(InterfaceState(scene_), 0.0);
			return;
		}
	}
	RCLCPP_WARN(LOGGER, "failed to acquire current PlanningScene");
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
