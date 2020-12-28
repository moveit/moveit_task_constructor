#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/simple_grasp.h>
#include <moveit/task_constructor/stages/pick.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;

void spawnObject() {
	moveit::planning_interface::PlanningSceneInterface psi;

	moveit_msgs::msg::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "base_link";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.53;
	o.primitive_poses[0].position.y = 0.05;
	o.primitive_poses[0].position.z = 0.84;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

TEST(PR2, pick) {
	Task t;

	Stage* initial_stage = new stages::CurrentState("current state");
	t.add(std::unique_ptr<Stage>(initial_stage));

	auto node = rclcpp::Node::make_shared("pr2");
	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>(node);
	pipeline->setPlannerId("RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = { { "left_arm", pipeline }, { "left_gripper", pipeline } };
	auto connect = std::make_unique<stages::Connect>("connect", planners);
	connect->properties().configureInitFrom(Stage::PARENT);
	t.add(std::move(connect));

	// grasp generator
	auto grasp_generator = new stages::GenerateGraspPose("generate grasp pose");
	grasp_generator->setAngleDelta(.2);
	grasp_generator->setPreGraspPose("open");
	grasp_generator->setGraspPose("closed");
	grasp_generator->setMonitoredStage(initial_stage);

	auto grasp = std::make_unique<stages::SimpleGrasp>(std::unique_ptr<MonitoringGenerator>(grasp_generator));
	grasp->setIKFrame(Eigen::Isometry3d::Identity(), "l_gripper_tool_frame");

	// pick stage
	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->setProperty("eef", std::string("left_gripper"));
	pick->setProperty("object", std::string("object"));
	geometry_msgs::msg::TwistStamped approach;
	approach.header.frame_id = "l_gripper_tool_frame";
	approach.twist.linear.x = 1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::msg::TwistStamped lift;
	lift.header.frame_id = "base_link";
	lift.twist.linear.z = 1.0;
	pick->setLiftMotion(lift, 0.03, 0.05);

	t.add(std::move(pick));

	try {
		spawnObject();
		t.plan();
	} catch (const InitStageException& e) {
		ADD_FAILURE() << "planning failed with exception" << std::endl << e << t;
	}

	auto solutions = t.solutions().size();
	EXPECT_GE(solutions, 5u);
	EXPECT_LE(solutions, 10u);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	// wait some time for move_group to come up
	rclcpp::sleep_for(std::chrono::seconds(5));

	return RUN_ALL_TESTS();
}
