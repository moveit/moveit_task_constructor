#include "models.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;
using namespace moveit::core;

constexpr double TAU{ 2 * M_PI };
constexpr double EPS{ 5e-5 };

// provide a basic test fixture that prepares a Task
template <typename Planner>
struct PandaMoveRelative : public testing::Test
{
	Task t;
	stages::MoveRelative* move;
	PlanningScenePtr scene;
	std::shared_ptr<Planner> planner;

	const JointModelGroup* group;

	PandaMoveRelative(std::shared_ptr<Planner> planner) : planner(planner) {
		t.setRobotModel(loadModel());

		group = t.getRobotModel()->getJointModelGroup("panda_arm");

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(t.getRobotModel()->getJointModelGroup("panda_arm"), "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move_relative = std::make_unique<stages::MoveRelative>("move", planner);
		move_relative->setGroup(group->getName());
		move = move_relative.get();
		t.add(std::move(move_relative));
	}
};

struct PandaMoveRelative_CartesianPath : public PandaMoveRelative<solvers::CartesianPath>
{
	PandaMoveRelative_CartesianPath() : PandaMoveRelative(std::make_shared<solvers::CartesianPath>()) {}
};

struct PandaMoveRelative_PilzIndustrialMotionPlanner : public PandaMoveRelative<solvers::PipelinePlanner>
{
	PandaMoveRelative_PilzIndustrialMotionPlanner()
	  : PandaMoveRelative(std::make_shared<solvers::PipelinePlanner>("pilz_industrial_motion_planner")) {}
};

moveit_msgs::CollisionObject createObject(const std::string& id, const geometry_msgs::Pose& pose) {
	moveit_msgs::CollisionObject co;
	co.header.frame_id = "panda_hand";
	co.operation = co.ADD;
	co.id = id;
	co.primitives.resize(1, [] {
		shape_msgs::SolidPrimitive p;
		p.type = p.SPHERE;
		p.dimensions.resize(1);
		p.dimensions[p.SPHERE_RADIUS] = 0.01;
		return p;
	}());

	co.pose = pose;
	return co;
}

moveit_msgs::CollisionObject createObject(const std::string& id) {
	geometry_msgs::Pose p;
	p.position.x = 0.1;
	p.orientation.w = 1.0;

	return createObject(id, p);
}

moveit_msgs::AttachedCollisionObject createAttachedObject(const std::string& id) {
	moveit_msgs::AttachedCollisionObject aco;
	aco.link_name = "panda_hand";
	aco.object = createObject(id);

	return aco;
}

void expect_const_position(const SolutionBaseConstPtr& solution, const std::string& tip,
                           const Eigen::Isometry3d& offset = Eigen::Isometry3d::Identity()) {
	const robot_trajectory::RobotTrajectory& t = *std::dynamic_pointer_cast<const SubTrajectory>(solution)->trajectory();
	const Eigen::Vector3d start_position = (t.getFirstWayPoint().getFrameTransform(tip) * offset).translation();
	for (size_t i = 0; i < t.getWayPointCount(); ++i) {
		const Eigen::Vector3d position = (t.getWayPoint(i).getFrameTransform(tip) * offset).translation();
		ASSERT_TRUE(start_position.isApprox(position, EPS))
		    << "Rotation must maintain position\n"
		    << i << ": " << start_position.transpose() << " != " << position.transpose();
	}
}

#define EXPECT_CONST_POSITION(...)                                \
	{                                                              \
		SCOPED_TRACE("expect_constant_position(" #__VA_ARGS__ ")"); \
		expect_const_position(__VA_ARGS__);                         \
	}

TEST_F(PandaMoveRelative_CartesianPath, cartesianRotateEEF) {
	move->setDirection([] {
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), group->getOnlyOneEndEffectorTip()->getName());
}

TEST_F(PandaMoveRelative_CartesianPath, cartesianCircular) {
	const std::string tip = "panda_hand";
	auto offset = Eigen::Translation3d(0, 0, 0.1);
	move->setIKFrame(offset, tip);
	move->setDirection([] {
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.x = TAU / 4.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), tip, Eigen::Isometry3d(offset));
}

TEST_F(PandaMoveRelative_CartesianPath, cartesianRotateAttachedIKFrame) {
	const std::string attached_object{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(attached_object));
	move->setIKFrame(attached_object);

	move->setDirection([] {
		geometry_msgs::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), attached_object);
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "move_relative_test");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	return RUN_ALL_TESTS();
}
