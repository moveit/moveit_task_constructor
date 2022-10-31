#include "models.h"
#include "stage_mockups.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/pilz.h>
#include <moveit/task_constructor/moveit_compat.h>

#include <moveit/planning_scene/planning_scene.h>
#include <pilz_industrial_motion_planner/joint_limits_aggregator.h>
#include <pilz_industrial_motion_planner/limits_container.h>

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
struct PandaMoveToPilz : public testing::Test
{
	Task t;
	stages::MoveTo* move_to;
	PlanningScenePtr scene;
	rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("panda_move_to");
	std::shared_ptr<solvers::Pilz> solver;

	PandaMoveToPilz() {
		t.loadRobotModel(node);

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"),
		                                                    "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		solver = std::make_shared<solvers::Pilz>();
		
		solver->setLimits(make_limits());

		auto move = std::make_unique<stages::MoveTo>("move", solver);
		move_to = move.get();
		move_to->setGroup("panda_arm");
		t.add(std::move(move));
	}

	pilz_industrial_motion_planner::LimitsContainer make_limits() {
		auto limits = pilz_industrial_motion_planner::LimitsContainer();

		auto cart_limits = pilz_industrial_motion_planner::CartesianLimit();
		cart_limits.setMaxRotationalVelocity(1.0);
		cart_limits.setMaxTranslationalVelocity(1.0);
		cart_limits.setMaxTranslationalAcceleration(1.0);
		cart_limits.setMaxTranslationalDeceleration(1.0);
		limits.setCartesianLimits(cart_limits);

		auto joint_limits = pilz_industrial_motion_planner::JointLimitsAggregator::getAggregatedLimits(
		    node, "robot_description_planning", t.getRobotModel()->getActiveJointModels());
		limits.setJointLimits(joint_limits);

		return limits;
	}
};

#define EXPECT_ONE_SOLUTION                \
	{                                       \
		EXPECT_TRUE(t.plan());               \
		EXPECT_EQ(t.solutions().size(), 1u); \
	}

geometry_msgs::msg::PoseStamped getFramePoseOfNamedState(RobotState state, std::string pose, std::string frame) {
	state.setToDefaultValues(state.getRobotModel()->getJointModelGroup("panda_arm"), pose);
	auto frame_eigen{ state.getFrameTransform(frame) };
	geometry_msgs::msg::PoseStamped p;
	p.header.frame_id = state.getRobotModel()->getModelFrame();
	p.pose = tf2::toMsg(frame_eigen);
	return p;
}

// 90 degree arc centered about the base
auto const test_goal_pose = [] {
	geometry_msgs::msg::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose.orientation.x = 0.7071;
	msg.pose.orientation.y = 0.7071;
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
	msg.pose.orientation.x = 0.7071;
	msg.pose.orientation.y = -0.7071;
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


TEST_F(PandaMoveToPilz, invalidPlanner) {
	EXPECT_FALSE(solver->setPlanner("BLAH"));
	move_to->setGoal(test_goal_pose);
	EXPECT_FALSE(t.plan());
}

TEST_F(PandaMoveToPilz, poseTargetPTP) {
	solver->setPlanner("PTP");
	move_to->setIKFrame("panda_hand");
	move_to->setGoal(test_goal_pose);
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveToPilz, poseTargetLIN) {
	solver->setPlanner("LIN");
	move_to->setIKFrame("panda_hand");
	move_to->setGoal(test_goal_pose);
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveToPilz, poseTargetCIRCCenter) {
	solver->setPlanner("CIRC");
	move_to->setIKFrame("panda_hand");
	move_to->setGoal(test_goal_pose);
	move_to->setCircularArcConstraint({"center", test_arc_center_pose});
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveToPilz, poseTargetCIRCInterim) {
	solver->setPlanner("CIRC");
	move_to->setIKFrame("panda_hand");
	move_to->setGoal(test_goal_pose);
	move_to->setCircularArcConstraint({"interim", test_arc_interim_pose});
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

TEST_F(PandaMoveToPilz, poseIKFrameAttachedTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	solver->setPlanner("LIN");
	move_to->setIKFrame(ATTACHED_OBJECT);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", ATTACHED_OBJECT));
	EXPECT_ONE_SOLUTION;
}

// If we don't have this, we also don't have subframe support
TEST_F(PandaMoveToPilz, poseIKFrameAttachedSubframeTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	const std::string IK_FRAME{ ATTACHED_OBJECT + "/subframe" };

	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	solver->setPlanner("LIN");
	move_to->setIKFrame(IK_FRAME);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", IK_FRAME));
	EXPECT_ONE_SOLUTION;
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	return RUN_ALL_TESTS();
}
