#include "models.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>

#include <moveit/planning_scene/planning_scene.h>

#include <tf2_eigen/tf2_eigen.h>

#include <moveit_msgs/RobotState.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/console.h>
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

	PandaMoveTo() {
		t.setRobotModel(loadModel());

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move = std::make_unique<stages::MoveTo>("move", std::make_shared<solvers::JointInterpolationPlanner>());
		move_to = move.get();
		move_to->setGroup("panda_arm");
		t.add(std::move(move));
	}
};

#define EXPECT_ONE_SOLUTION                \
	{                                       \
		EXPECT_TRUE(t.plan());               \
		EXPECT_EQ(t.solutions().size(), 1u); \
	}

TEST_F(PandaMoveTo, namedTarget) {
	move_to->setGoal("extended");
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, mapTarget) {
	move_to->setGoal(std::map<std::string, double>{ { "panda_joint1", TAU / 8 }, { "panda_joint2", TAU / 8 } });
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, stateTarget) {
	move_to->setGoal([]() {
		moveit_msgs::RobotState state;
		state.is_diff = true;
		state.joint_state.name = { "panda_joint1", "panda_joint2" };
		state.joint_state.position = { TAU / 8, TAU / 8 };
		return state;
	}());
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, pointTarget) {
	move_to->setGoal([this]() {
		RobotState state{ scene->getCurrentState() };
		state.setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "extended");
		auto frame{ state.getFrameTransform("panda_link8") };
		geometry_msgs::PointStamped point;
		point.header.frame_id = scene->getPlanningFrame();
		point.point = tf2::toMsg(Eigen::Vector3d{ frame.translation() });
		return point;
	}());
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, poseTarget) {
	move_to->setGoal([this]() {
		RobotState state{ scene->getCurrentState() };
		state.setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "extended");
		auto frame{ state.getFrameTransform("panda_link8") };
		geometry_msgs::PoseStamped pose;
		pose.header.frame_id = scene->getPlanningFrame();
		pose.pose = tf2::toMsg(frame);
		return pose;
	}());
	EXPECT_ONE_SOLUTION;
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "move_to_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	return RUN_ALL_TESTS();
}
