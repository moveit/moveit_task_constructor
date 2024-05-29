#include "models.h"
#include "stage_mockups.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/connect.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>

#include <moveit/planning_scene/planning_scene.h>

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

#include <moveit_msgs/msg/robot_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/logging.hpp>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;
using namespace moveit::core;

constexpr double TAU{ 2 * M_PI };

// provide a basic test fixture that prepares a Task
struct PandaMoveTo : public testing::Test
{
	Task t;
	stages::MoveTo* move_to;
	PlanningScenePtr scene;
	rclcpp::Node::SharedPtr node;

	PandaMoveTo() {
		node = rclcpp::Node::make_shared("panda_move_to");
		t.loadRobotModel(node);

		auto group = t.getRobotModel()->getJointModelGroup("panda_arm");

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(group, "extended");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move = std::make_unique<stages::MoveTo>("move", std::make_shared<solvers::JointInterpolationPlanner>());
		move_to = move.get();
		move_to->setGroup(group->getName());
		t.add(std::move(move));
	}
};

#define EXPECT_ONE_SOLUTION                \
	{                                       \
		EXPECT_TRUE(t.plan());               \
		EXPECT_EQ(t.solutions().size(), 1u); \
	}

TEST_F(PandaMoveTo, namedTarget) {
	move_to->setGoal("ready");
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, mapTarget) {
	move_to->setGoal(std::map<std::string, double>{ { "panda_joint1", TAU / 8 }, { "panda_joint2", TAU / 8 } });
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, stateTarget) {
	move_to->setGoal([] {
		moveit_msgs::msg::RobotState state;
		state.is_diff = true;
		state.joint_state.name = { "panda_joint1", "panda_joint2" };
		state.joint_state.position = { TAU / 8, TAU / 8 };
		return state;
	}());
	EXPECT_ONE_SOLUTION;
}

geometry_msgs::msg::PoseStamped getFramePoseOfNamedState(RobotState state, const std::string& pose,
                                                         const std::string& frame) {
	state.setToDefaultValues(state.getRobotModel()->getJointModelGroup("panda_arm"), pose);
	auto frame_eigen{ state.getFrameTransform(frame) };
	geometry_msgs::msg::PoseStamped p;
	p.header.frame_id = state.getRobotModel()->getModelFrame();
	p.pose = tf2::toMsg(frame_eigen);
	return p;
}

TEST_F(PandaMoveTo, pointTarget) {
	geometry_msgs::msg::PoseStamped pose{ getFramePoseOfNamedState(scene->getCurrentState(), "ready", "panda_link8") };

	geometry_msgs::msg::PointStamped point;
	point.header = pose.header;
	point.point = pose.pose.position;
	move_to->setGoal(point);

	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, poseTarget) {
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", "panda_link8"));
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, poseIKFrameLinkTarget) {
	const std::string ik_frame{ "panda_hand" };
	move_to->setIKFrame(ik_frame);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", ik_frame));
	EXPECT_ONE_SOLUTION;
}

moveit_msgs::msg::AttachedCollisionObject createAttachedObject(const std::string& id) {
	moveit_msgs::msg::AttachedCollisionObject aco;
	aco.link_name = "panda_hand";
	aco.object.header.frame_id = aco.link_name;
	aco.object.operation = aco.object.ADD;
	aco.object.id = id;
	aco.object.primitives.resize(1, [] {
		shape_msgs::msg::SolidPrimitive p;
		p.type = p.SPHERE;
		p.dimensions.resize(1);
		p.dimensions[p.SPHERE_RADIUS] = 0.01;
		return p;
	}());
	aco.object.pose.position.z = 0.2;
	aco.object.pose.orientation.w = 1.0;
	aco.object.subframe_names.resize(1, "subframe");
	aco.object.subframe_poses.resize(1, [] {
		geometry_msgs::msg::Pose p;
		p.orientation.w = 1.0;
		return p;
	}());
	return aco;
}

TEST_F(PandaMoveTo, poseIKFrameAttachedTarget) {
	const std::string attached_object{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(attached_object));

	move_to->setIKFrame(attached_object);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", attached_object));
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, poseIKFrameAttachedSubframeTarget) {
	const std::string attached_object{ "attached_object" };
	const std::string ik_frame{ attached_object + "/subframe" };

	scene->processAttachedCollisionObjectMsg(createAttachedObject(attached_object));

	move_to->setIKFrame(ik_frame);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", ik_frame));
	EXPECT_ONE_SOLUTION;
}

// Using a Cartesian interpolation planner targeting a joint-space goal, which is
// transformed into a Cartesian goal by FK, should fail if the two poses are on different
// IK solution branches. In this case, the end-state, although reaching the Cartesian goal,
// will strongly deviate from the joint-space goal.
TEST(Panda, connectCartesianBranchesFails) {
	Task t;
	t.loadRobotModel(rclcpp::Node::make_shared("panda_move_to"));
	auto scene = std::make_shared<PlanningScene>(t.getRobotModel());
	scene->getCurrentStateNonConst().setToDefaultValues();
	scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "ready");
	t.add(std::make_unique<stages::FixedState>("start", scene));

	stages::Connect::GroupPlannerVector planner = { { "panda_arm", std::make_shared<solvers::CartesianPath>() } };
	t.add(std::make_unique<stages::Connect>("connect", planner));

	// target an elbow-left instead of an elbow-right solution (different solution branch)
	scene = scene->diff();
	scene->getCurrentStateNonConst().setJointGroupPositions(
	    "panda_arm", std::vector<double>({ 2.72, 0.78, -2.63, -2.35, 0.36, 1.57, 0.48 }));

	t.add(std::make_unique<stages::FixedState>("end", scene));
	EXPECT_FALSE(t.plan());
	EXPECT_STREQ(t.findChild("connect")->failures().front()->comment().c_str(),
	             "Trajectory end-point deviates too much from goal state");
}

// This test requires a running rosmaster
TEST(Task, taskMoveConstructor) {
	auto create_task = [] {
		moveit::core::RobotModelConstPtr robot_model = getModel();
		Task t("first");
		t.setRobotModel(robot_model);
		auto ref = new stages::FixedState("fixed");
		auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
		ref->setState(scene);

		t.add(Stage::pointer(ref));
		t.add(std::make_unique<ConnectMockup>());
		t.add(std::make_unique<MonitoringGeneratorMockup>(ref));
		return t;
	};

	Task t;
	t = create_task();

	try {
		t.init();
		EXPECT_TRUE(t.plan(1));
	} catch (const InitStageException& e) {
		ADD_FAILURE() << "InitStageException:" << std::endl << e << t;
	}
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	return RUN_ALL_TESTS();
}
