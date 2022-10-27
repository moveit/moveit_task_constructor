#include "models.h"
#include "stage_mockups.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_circle.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/moveit_compat.h>

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
struct PandaMoveCircle : public testing::Test
{
	Task t;
	stages::MoveCircle* move_circle;
	PlanningScenePtr scene;
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("panda_move_circle");

	PandaMoveCircle() {
		t.loadRobotModel(node);

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move = std::make_unique<stages::MoveCircle>("move circle");
		move_circle = move.get();
		move_circle->setGroup("panda_arm");
		move_circle->setLink("panda_hand");
		t.add(std::move(move));
	}
};

#define EXPECT_ONE_SOLUTION                \
	{                                       \
		EXPECT_TRUE(t.plan());               \
		EXPECT_EQ(t.solutions().size(), 1u); \
	}

// 90 degree arc centered about the base
auto const test_goal_pose = [] {
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose.orientation.x = 0.707;
	msg.pose.orientation.y = 0.707;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 0.0;
	msg.pose.position.x = 0.0;
	msg.pose.position.y = 0.3;
	msg.pose.position.z = 0.59;
	return msg;
}();

// The center of the base, at the initial height of the hand
auto const test_arc_center_pose = [] {
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose.orientation.x = 0.707;
	msg.pose.orientation.y = -0.707;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 0.0;
	msg.pose.position.x = 0.0;
	msg.pose.position.y = 0.0;
	msg.pose.position.z = 0.59;
	return msg;
}();

// Midpoint selected halfway (45 degrees) along the arc.
auto const test_arc_interim_pose = [] {
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose.orientation.x = 0.9381665;
	msg.pose.orientation.y = 0.3461834;
	msg.pose.orientation.z = 0.0;
	msg.pose.orientation.w = 0.0;
	msg.pose.position.x = 0.21708;
	msg.pose.position.y = 0.21708;
	msg.pose.position.z = 0.59;
	return msg;
}();

/// Plan to a goal pose using the arc center point constraint.
TEST_F(PandaMoveCircle, planGoalCenter) {
	move_circle->setGoal(test_goal_pose);
	move_circle->setArcConstraint({ "center", test_arc_center_pose });
	EXPECT_ONE_SOLUTION;
}

/// Plan to a goal pose using the arc interim point constraint.
TEST_F(PandaMoveCircle, planGoalInterim) {
	move_circle->setGoal(test_goal_pose);
	move_circle->setArcConstraint({"interim", test_arc_interim_pose});
	EXPECT_ONE_SOLUTION;
}

/// Plan to a goal pose using an invalid constraint.
TEST_F(PandaMoveCircle, planInvalidConstraint) {
	move_circle->setGoal(test_goal_pose);
	move_circle->setArcConstraint({"invalid", test_arc_center_pose});
	EXPECT_FALSE(t.plan());
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
	aco.object.primitive_poses.resize(1);
	aco.object.pose.position.z = 0.2;
	aco.object.pose.orientation.w = 1.0;
	// If we don't have this, we also don't have subframe support
	aco.object.subframe_names.resize(1, "subframe");
	aco.object.subframe_poses.resize(1, [] {
		geometry_msgs::msg::Pose p;
		p.orientation.w = 1.0;
		return p;
	}());
	return aco;
}

geometry_msgs::msg::PoseStamped getFramePoseOfNamedState(RobotState state, std::string pose, std::string frame) {
	state.setToDefaultValues(state.getRobotModel()->getJointModelGroup("panda_arm"), pose);
	auto frame_eigen{ state.getFrameTransform(frame) };
	geometry_msgs::msg::PoseStamped p;
	p.header.frame_id = state.getRobotModel()->getModelFrame();
	p.pose = tf2::toMsg(frame_eigen);
	return p;
}

TEST_F(PandaMoveCircle, poseIKFrameAttachedTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	move_circle->setLink(ATTACHED_OBJECT);

	auto goal_pose = test_goal_pose;
	goal_pose.header.frame_id = "world";
	goal_pose.pose.position.z -= 0.2;
	move_circle->setGoal(goal_pose);

	auto center_pose = test_arc_center_pose;
	center_pose.header.frame_id = "world";
	center_pose.pose.position.z -= 0.2;
	move_circle->setArcConstraint({"center", center_pose});

	EXPECT_ONE_SOLUTION;
}

// If we don't have this, we also don't have subframe support
TEST_F(PandaMoveCircle, poseIKFrameAttachedSubframeTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	const std::string LINK_FRAME{ ATTACHED_OBJECT + "/subframe" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	move_circle->setLink(LINK_FRAME);

	auto goal_pose = test_goal_pose;
	goal_pose.header.frame_id = "world";
	goal_pose.pose.position.z -= 0.2;
	move_circle->setGoal(goal_pose);

	auto center_pose = test_arc_center_pose;
	center_pose.header.frame_id = "world";
	center_pose.pose.position.z -= 0.2;
	move_circle->setArcConstraint({"center", center_pose});

	EXPECT_ONE_SOLUTION;
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	return RUN_ALL_TESTS();
}
