#include "models.h"
#include "stage_mockups.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/task_constructor/moveit_compat.h>

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
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"),
		                                                    "extended");
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
	move_to->setGoal("ready");
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, mapTarget) {
	move_to->setGoal(std::map<std::string, double>{ { "panda_joint1", TAU / 8 }, { "panda_joint2", TAU / 8 } });
	EXPECT_ONE_SOLUTION;
}

TEST_F(PandaMoveTo, stateTarget) {
	move_to->setGoal([] {
		moveit_msgs::RobotState state;
		state.is_diff = true;
		state.joint_state.name = { "panda_joint1", "panda_joint2" };
		state.joint_state.position = { TAU / 8, TAU / 8 };
		return state;
	}());
	EXPECT_ONE_SOLUTION;
}

geometry_msgs::PoseStamped getFramePoseOfNamedState(RobotState state, std::string pose, std::string frame) {
	state.setToDefaultValues(state.getRobotModel()->getJointModelGroup("panda_arm"), pose);
	auto frame_eigen{ state.getFrameTransform(frame) };
	geometry_msgs::PoseStamped p;
	p.header.frame_id = state.getRobotModel()->getModelFrame();
	p.pose = tf2::toMsg(frame_eigen);
	return p;
}

TEST_F(PandaMoveTo, pointTarget) {
	geometry_msgs::PoseStamped pose{ getFramePoseOfNamedState(scene->getCurrentState(), "ready", "panda_link8") };

	geometry_msgs::PointStamped point;
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
	const std::string IK_FRAME{ "panda_hand" };
	move_to->setIKFrame(IK_FRAME);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", IK_FRAME));
	EXPECT_ONE_SOLUTION;
}

moveit_msgs::AttachedCollisionObject createAttachedObject(const std::string& id) {
	moveit_msgs::AttachedCollisionObject aco;
	aco.link_name = "panda_hand";
	aco.object.header.frame_id = aco.link_name;
	aco.object.operation = aco.object.ADD;
	aco.object.id = id;
	aco.object.primitives.resize(1, [] {
		shape_msgs::SolidPrimitive p;
		p.type = p.SPHERE;
		p.dimensions.resize(1);
		p.dimensions[p.SPHERE_RADIUS] = 0.01;
		return p;
	}());
#if MOVEIT_HAS_OBJECT_POSE
	aco.object.pose.position.z = 0.2;
	aco.object.pose.orientation.w = 1.0;
#else
	aco.object.primitive_poses.resize(1);
	aco.object.primitive_poses[0].position.z = 0.2;
	aco.object.primitive_poses[0].orientation.w = 1.0;
#endif
#if MOVEIT_HAS_STATE_RIGID_PARENT_LINK
	// If we don't have this, we also don't have subframe support
	aco.object.subframe_names.resize(1, "subframe");
	aco.object.subframe_poses.resize(1, [] {
		geometry_msgs::Pose p;
		p.orientation.w = 1.0;
		return p;
	}());
#endif
	return aco;
}

TEST_F(PandaMoveTo, poseIKFrameAttachedTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	move_to->setIKFrame(ATTACHED_OBJECT);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", ATTACHED_OBJECT));
	EXPECT_ONE_SOLUTION;
}

#if MOVEIT_HAS_STATE_RIGID_PARENT_LINK
// If we don't have this, we also don't have subframe support
TEST_F(PandaMoveTo, poseIKFrameAttachedSubframeTarget) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	const std::string IK_FRAME{ ATTACHED_OBJECT + "/subframe" };

	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));

	move_to->setIKFrame(IK_FRAME);
	move_to->setGoal(getFramePoseOfNamedState(scene->getCurrentState(), "ready", IK_FRAME));
	EXPECT_ONE_SOLUTION;
}
#endif

// This test require a running rosmaster
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

	// Segfaults when introspection is enabled
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
	ros::init(argc, argv, "move_to_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	return RUN_ALL_TESTS();
}
