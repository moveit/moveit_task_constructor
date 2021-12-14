#include "models.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/moveit_compat.h>

#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;
using namespace moveit::core;

constexpr double TAU{ 2 * M_PI };
constexpr double EPS{ 1e-6 };

// provide a basic test fixture that prepares a Task
struct PandaMoveRelative : public testing::Test
{
	Task t;
	stages::MoveRelative* move;
	PlanningScenePtr scene;

	const JointModelGroup* group;

	PandaMoveRelative() {
		t.setRobotModel(loadModel());

		group = t.getRobotModel()->getJointModelGroup("panda_arm");

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move_relative = std::make_unique<stages::MoveRelative>("move", std::make_shared<solvers::CartesianPath>());
		move_relative->setGroup(group->getName());
		move = move_relative.get();
		t.add(std::move(move_relative));
	}
};

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

	geometry_msgs::Pose p;
	p.position.x = 0.1;
	p.orientation.w = 1.0;
#if MOVEIT_HAS_OBJECT_POSE
	aco.object.pose = p;
#else
	aco.object.primitive_poses.resize(1, p);
	aco.object.primitive_poses[0] = p;
#endif
	return aco;
}

inline auto position(const PlanningSceneConstPtr& scene, const std::string& frame) {
	return scene->getFrameTransform(frame).translation();
}

TEST_F(PandaMoveRelative, cartesianRotateEEF) {
	move->setDirection([] {
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";

	const auto& tip_name{ group->getOnlyOneEndEffectorTip()->getName() };
	const auto start_eef_position{ position(scene, tip_name) };
	const auto end_eef_position{ position(move->solutions().front()->end()->scene(), tip_name) };

	EXPECT_TRUE(start_eef_position.isApprox(end_eef_position, EPS))
	    << "Cartesian rotation unexpectedly changed position of '" << tip_name << "' (must only change orientation)\n"
	    << start_eef_position << "\nvs\n"
	    << end_eef_position;
}

TEST_F(PandaMoveRelative, cartesianRotateAttachedIKFrame) {
	const std::string ATTACHED_OBJECT{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(ATTACHED_OBJECT));
	move->setIKFrame(ATTACHED_OBJECT);

	move->setDirection([] {
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan());

	const auto start_eef_position{ position(scene, ATTACHED_OBJECT) };
	const auto end_eef_position{ position(move->solutions().front()->end()->scene(), ATTACHED_OBJECT) };

	EXPECT_TRUE(start_eef_position.isApprox(end_eef_position, EPS))
	    << "Cartesian rotation unexpectedly changed position of ik frame (must only change orientation)\n"
	    << start_eef_position << "\nvs\n"
	    << end_eef_position;
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "move_relative_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	return RUN_ALL_TESTS();
}
