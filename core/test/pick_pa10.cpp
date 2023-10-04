#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/generate_grasp_pose.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/stages/fix_collision_objects.h>

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;

void spawnObject(const planning_scene::PlanningScenePtr& scene) {
	moveit_msgs::msg::CollisionObject o;
	o.id = "object";
	o.header.frame_id = "world";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.3;
	o.primitive_poses[0].position.y = 0.23;
	o.primitive_poses[0].position.z = 0.10;
	o.primitive_poses[0].orientation.w = 1.0;
	o.primitives.resize(1);
	o.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
	o.primitives[0].dimensions.resize(2);
	o.primitives[0].dimensions[0] = 0.23;
	o.primitives[0].dimensions[1] = 0.03;
	scene->processCollisionObjectMsg(o);
}

TEST(PA10, pick) {
	Task t;
	t.stages()->setName("pick");

	auto node = rclcpp::Node::make_shared("pa10");
	t.loadRobotModel(node);
	// define global properties used by most stages
	t.setProperty("group", std::string("left_arm"));
	t.setProperty("eef", std::string("la_tool_mount"));
	t.setProperty("gripper", std::string("left_hand"));

	auto pipeline = std::make_shared<solvers::PipelinePlanner>(node, "ompl", "RRTConnectkConfigDefault");
	auto cartesian = std::make_shared<solvers::CartesianPath>();

	Stage* initial_stage = nullptr;
	// create a fixed initial scene
	{
		auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
		auto& state = scene->getCurrentStateNonConst();
		state.setToDefaultValues();  // initialize state
		state.setToDefaultValues(state.getJointModelGroup("left_arm"), "home");
		state.setToDefaultValues(state.getJointModelGroup("right_arm"), "home");
		state.update();
		spawnObject(scene);

		auto initial = std::make_unique<stages::FixedState>();
		initial->setState(scene);
		t.add(std::move(initial));
	}
	{
		auto stage = std::make_unique<stages::FixCollisionObjects>();
		stage->restrictDirection(stages::MoveTo::FORWARD);
		stage->setMaxPenetration(0.04);
		initial_stage = stage.get();
		t.add(std::move(stage));
	}

	{
		stages::Connect::GroupPlannerVector planners = { { "left_hand", pipeline }, { "left_arm", pipeline } };
		auto move = std::make_unique<stages::Connect>("connect", planners);
		move->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("approach object", cartesian);
		move->restrictDirection(stages::MoveRelative::BACKWARD);
		move->properties().configureInitFrom(Stage::PARENT);
		move->properties().set("marker_ns", std::string("approach"));
		move->setIKFrame("lh_tool_frame");
		move->setMinMaxDistance(0.05, 0.1);

		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "lh_tool_frame";
		direction.vector.z = 1;
		move->setDirection(direction);
		t.add(std::move(move));
	}

	{
		auto gengrasp = std::make_unique<stages::GenerateGraspPose>("generate grasp pose");
		gengrasp->properties().configureInitFrom(Stage::PARENT);
		gengrasp->setPreGraspPose("open");
		gengrasp->setObject("object");
		gengrasp->setAngleDelta(M_PI / 10.);
		gengrasp->setMonitoredStage(initial_stage);

		auto filter = std::make_unique<stages::PredicateFilter>("filtered");
		gengrasp->properties().exposeTo(filter->properties(), { "eef" });
		filter->properties().configureInitFrom(Stage::PARENT);
		filter->insert(std::move(gengrasp));
		filter->setPredicate([](const SolutionBase& s, std::string& comment) {
			bool accept = s.cost() < 2;
			if (!accept)
				comment += " (rejected)";
			return accept;
		});

		auto ik = std::make_unique<stages::ComputeIK>("compute ik", std::move(filter));
		PropertyMap& props = ik->properties();
		props.configureInitFrom(Stage::PARENT, { "group", "eef", "default_pose" });
		props.configureInitFrom(Stage::INTERFACE, { "target_pose" });  // derived from child's solution
		ik->setIKFrame(Eigen::Translation3d(0, 0, .05) * Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d::UnitY()),
		               "lh_tool_frame");
		ik->setMaxIKSolutions(1);
		t.add(std::move(ik));
	}

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("allow object collision");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);

		move->allowCollisions(
		    "object", t.getRobotModel()->getJointModelGroup("left_hand")->getLinkModelNamesWithCollisionGeometry(), true);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveTo>("close gripper", pipeline);
		move->restrictDirection(stages::MoveTo::FORWARD);
		move->properties().property("group").configureInitFrom(Stage::PARENT, "gripper");
		move->setGoal("closed");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::ModifyPlanningScene>("attach object");
		move->restrictDirection(stages::ModifyPlanningScene::FORWARD);
		move->attachObject("object", "lh_tool_frame");
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("lift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, { "group" });
		move->setMinMaxDistance(0.03, 0.05);
		move->properties().set("marker_ns", std::string("lift"));
		move->setIKFrame("lh_tool_frame");

		geometry_msgs::msg::Vector3Stamped direction;
		direction.header.frame_id = "world";
		direction.vector.z = 1;
		move->setDirection(direction);
		t.add(std::move(move));
	}

	{
		auto move = std::make_unique<stages::MoveRelative>("shift object", cartesian);
		move->properties().configureInitFrom(Stage::PARENT, { "group" });
		move->setMinMaxDistance(0.1, 0.2);
		move->properties().set("marker_ns", std::string("lift"));
		move->setIKFrame("lh_tool_frame");

		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "object";
		twist.twist.linear.y = 1;
		twist.twist.angular.y = 2;
		move->setDirection(twist);
		t.add(std::move(move));
	}

	try {
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

	return RUN_ALL_TESTS();
}
