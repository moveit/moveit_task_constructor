/*********************************************************************
 * Copyright (c) 2019 Bielefeld University
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

/* Author: Robert Haschke
   Desc:   Planning a simple sequence of Cartesian motions
*/

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/container.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>

using namespace moveit::task_constructor;

std::unique_ptr<SerialContainer> createModule(const std::string& group) {
	auto c = std::make_unique<SerialContainer>("Cartesian Path");
	c->setProperty("group", group);

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();
	// create joint interpolation "planner"
	auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT, { "group" });
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.2;
		stage->setDirection(direction);
		c->insert(std::move(stage));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("y -0.3", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT);
		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.y = -0.3;
		stage->setDirection(direction);
		c->insert(std::move(stage));
	}

	{  // rotate about TCP
		auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian);
		stage->properties().configureInitFrom(Stage::PARENT);
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = M_PI / 4.;
		stage->setDirection(twist);
		c->insert(std::move(stage));
	}

	{  // move back to ready pose
		auto stage = std::make_unique<stages::MoveTo>("moveTo ready", joint_interpolation);
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setGoal("ready");
		c->insert(std::move(stage));
	}
	return c;
}

Task createTask(const rclcpp::Node::SharedPtr& node) {
	Task t;
	t.loadRobotModel(node);
	t.stages()->setName("Reusable Containers");
	t.add(std::make_unique<stages::CurrentState>("current"));

	const std::string group = "panda_arm";
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));
	t.add(createModule(group));

	return t;
}

int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("mtc_tutorial");
	std::thread spinning_thread([node] { rclcpp::spin(node); });

	auto task = createTask(node);
	try {
		if (task.plan())
			task.introspection().publishSolution(*task.solutions().front());
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	// keep alive for interactive inspection in rviz
	spinning_thread.join();
	return 0;
}
