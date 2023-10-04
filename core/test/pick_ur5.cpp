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
	o.header.frame_id = "table_top";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = -0.2;
	o.primitive_poses[0].position.y = 0.13;
	o.primitive_poses[0].position.z = 0.12;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	psi.applyCollisionObject(o);
}

TEST(UR5, pick) {
	Task t;

	Stage* initial_stage = nullptr;
	auto initial = std::make_unique<stages::CurrentState>("current state");
	initial_stage = initial.get();
	t.add(std::move(initial));

	auto node = rclcpp::Node::make_shared("ur5");
	// planner used for connect
	auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
	// connect to pick
	stages::Connect::GroupPlannerVector planners = { { "arm", pipeline }, { "gripper", pipeline } };
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
	grasp->setIKFrame(Eigen::Translation3d(.03, 0, 0), "s_model_tool0");
	grasp->setMaxIKSolutions(8);

	auto pick = std::make_unique<stages::Pick>(std::move(grasp));
	pick->setProperty("eef", std::string("gripper"));
	pick->setProperty("object", std::string("object"));
	geometry_msgs::msg::TwistStamped approach;
	approach.header.frame_id = "s_model_tool0";
	approach.twist.linear.x = 1.0;
	pick->setApproachMotion(approach, 0.03, 0.1);

	geometry_msgs::msg::TwistStamped lift;
	lift.header.frame_id = "world";
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
	EXPECT_GE(solutions, 15u);
	EXPECT_LE(solutions, 60u);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	// wait some time for move_group to come up
	rclcpp::sleep_for(std::chrono::seconds(5));

	return RUN_ALL_TESTS();
}
