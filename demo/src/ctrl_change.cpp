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

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>

#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_task_constructor_msgs/ExecuteTaskSolutionAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace moveit::task_constructor;

Task createTask() {
	Task t;
	t.stages()->setName("Cartesian Path");

	const std::string group = "panda_arm";

	// create Cartesian interpolation "planner" to be used in stages
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	// start from a fixed robot state
	t.loadRobotModel();
	auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
	{
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues(state.getJointModelGroup(group), "ready");

		auto fixed = std::make_unique<stages::FixedState>("initial state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	{
		auto stage = std::make_unique<stages::MoveRelative>("x +0.2", cartesian);
		stage->setGroup(group);
		stage->setControllers({ "fake_via_panda_arm_controller" });
		geometry_msgs::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.x = 0.2;
		stage->setDirection(direction);
		t.add(std::move(stage));
	}

	{
		auto serial = std::make_unique<SerialContainer>();
		serial->setControllers({ "fake_last_panda_arm_controller" });
		// TODO smth like  serial->setForwardedProperties({"controllers"}); but for children instead InterfaceStates

		{
			auto stage = std::make_unique<stages::MoveRelative>("y -0.3", cartesian);
			stage->setGroup(group);
			//			stage->properties().configureInitFrom(Stage::PARENT, { "controllers" });
			geometry_msgs::Vector3Stamped direction;
			direction.header.frame_id = "world";
			direction.vector.y = -0.3;
			stage->setDirection(direction);
			serial->insert(std::move(stage));
		}

		{  // rotate about TCP
			auto stage = std::make_unique<stages::MoveRelative>("rz +45°", cartesian);
			stage->setGroup(group);
			//			stage->properties().configureInitFrom(Stage::PARENT, { "controllers" });
			geometry_msgs::TwistStamped twist;
			twist.header.frame_id = "world";
			twist.twist.angular.z = M_PI / 4.;
			stage->setDirection(twist);
			serial->insert(std::move(stage));
		}

		t.add(std::move(serial));
	}

	{  // perform a Cartesian motion, defined as a relative offset in joint space
		auto stage = std::make_unique<stages::MoveRelative>("joint offset", cartesian);
		stage->setGroup(group);
		std::map<std::string, double> offsets = { { "panda_joint1", M_PI / 6. }, { "panda_joint3", -M_PI / 6 } };
		stage->setDirection(offsets);
		t.add(std::move(stage));
	}

	{  // move from reached state back to the original state, using joint interpolation
		auto joint_interpolation = std::make_shared<solvers::JointInterpolationPlanner>();
		stages::Connect::GroupPlannerVector planners = { { group, joint_interpolation } };
		auto connect = std::make_unique<stages::Connect>("connect", planners);
		t.add(std::move(connect));
	}

	{  // final state is original state again
		auto fixed = std::make_unique<stages::FixedState>("final state");
		fixed->setState(scene);
		t.add(std::move(fixed));
	}

	return t;
}

std::string buildControllerString(moveit_task_constructor_msgs::SubTrajectory st) {
	if (st.controller_names.empty())
		return "";

	std::stringstream ss;
	for (auto cn : st.controller_names) {
		ss << cn.data << ";";
	}
	return ss.str();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "mtc_tutorial");
	// run an asynchronous spinner to communicate with the move_group node and rviz
	ros::AsyncSpinner spinner(4);
	spinner.start();

	auto task = createTask();
	try {
		if (task.plan()) {
			task.introspection().publishSolution(*task.solutions().front());
		} else {
			std::cerr << "planning failed" << std::endl;
			return 0;
		}
	} catch (const InitStageException& ex) {
		std::cerr << "planning failed with exception" << std::endl << ex << task;
	}

	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task.solutions().front()->fillMessage(execute_goal.solution);

	printf("SubTrajectories: %zu\n", execute_goal.solution.sub_trajectory.size());

	for (int i = 0; i < execute_goal.solution.sub_trajectory.size(); i++) {
		auto st = execute_goal.solution.sub_trajectory[i];
		printf("\tST%d with %zu points and controllers \"%s\"\n", i, st.trajectory.joint_trajectory.points.size(),
		       buildControllerString(st).c_str());
	}

	actionlib::SimpleActionClient<moveit_task_constructor_msgs::ExecuteTaskSolutionAction> ac("execute_task_solution",
	                                                                                          false);
	ac.waitForServer();

	ac.sendGoal(execute_goal);
	ac.waitForResult();

	moveit_msgs::MoveItErrorCodes execute_result = ac.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED("move_cart", "Task execution failed and returned: " << ac.getState().toString());
		return false;
	}

	ros::waitForShutdown();  // keep alive for interactive inspection in rviz
	return 0;
}
