#include "models.h"

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/solvers/cartesian_path.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <gtest/gtest.h>

#ifndef TYPED_TEST_SUITE  // for Melodic
#define TYPED_TEST_SUITE(SUITE, TYPES) TYPED_TEST_CASE(SUITE, TYPES)
#endif

using namespace moveit::task_constructor;
using namespace planning_scene;
using namespace moveit::core;

constexpr double TAU{ 2 * M_PI };
constexpr double EPS{ 5e-5 };

template <typename Planner>
std::shared_ptr<Planner> create(const rclcpp::Node::SharedPtr&);
template <>
solvers::CartesianPathPtr create<solvers::CartesianPath>(const rclcpp::Node::SharedPtr&) {
	return std::make_shared<solvers::CartesianPath>();
}
template <>
solvers::PipelinePlannerPtr create<solvers::PipelinePlanner>(const rclcpp::Node::SharedPtr& node) {
	auto p = std::make_shared<solvers::PipelinePlanner>(node, "pilz_industrial_motion_planner", "LIN");
	p->setProperty("max_velocity_scaling_factor", 0.1);
	p->setProperty("max_acceleration_scaling_factor", 0.1);
	return p;
}

// provide a basic test fixture that prepares a Task
template <typename Planner = solvers::CartesianPath>
struct PandaMoveRelative : public testing::Test
{
	Task t;
	stages::MoveRelative* move;
	PlanningScenePtr scene;
	std::shared_ptr<Planner> planner;

	const JointModelGroup* group;

	PandaMoveRelative() : planner(create<Planner>(rclcpp::Node::make_shared("panda_move_relative"))) {
		t.loadRobotModel(rclcpp::Node::make_shared("panda_move_relative"));

		group = t.getRobotModel()->getJointModelGroup("panda_arm");

		scene = std::make_shared<PlanningScene>(t.getRobotModel());
		scene->getCurrentStateNonConst().setToDefaultValues();
		scene->getCurrentStateNonConst().setToDefaultValues(group, "ready");
		t.add(std::make_unique<stages::FixedState>("start", scene));

		auto move_relative = std::make_unique<stages::MoveRelative>("move", planner);
		move_relative->setGroup(group->getName());
		move = move_relative.get();
		t.add(std::move(move_relative));
	}
};
using PandaMoveRelativeCartesian = PandaMoveRelative<solvers::CartesianPath>;

moveit_msgs::msg::CollisionObject createObject(const std::string& id, const geometry_msgs::msg::Pose& pose) {
	moveit_msgs::msg::CollisionObject co;
	co.header.frame_id = "panda_hand";
	co.operation = co.ADD;
	co.id = id;
	co.primitives.resize(1, [] {
		shape_msgs::msg::SolidPrimitive p;
		p.type = p.SPHERE;
		p.dimensions.resize(1);
		p.dimensions[p.SPHERE_RADIUS] = 0.01;
		return p;
	}());

	co.pose = pose;
	return co;
}

moveit_msgs::msg::CollisionObject createObject(const std::string& id) {
	geometry_msgs::msg::Pose p;
	p.position.x = 0.1;
	p.orientation.w = 1.0;

	return createObject(id, p);
}

moveit_msgs::msg::AttachedCollisionObject createAttachedObject(const std::string& id) {
	moveit_msgs::msg::AttachedCollisionObject aco;
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

#define EXPECT_CONST_POSITION(...)                             \
	{                                                           \
		SCOPED_TRACE("expect_const_position(" #__VA_ARGS__ ")"); \
		expect_const_position(__VA_ARGS__);                      \
	}

TEST_F(PandaMoveRelativeCartesian, cartesianRotateEEF) {
	move->setDirection([] {
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), group->getOnlyOneEndEffectorTip()->getName());
}

TEST_F(PandaMoveRelativeCartesian, cartesianCircular) {
	const std::string tip = "panda_hand";
	auto offset = Eigen::Translation3d(0, 0, 0.1);
	move->setIKFrame(offset, tip);
	move->setDirection([] {
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.x = TAU / 4.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), tip, Eigen::Isometry3d(offset));
}

TEST_F(PandaMoveRelativeCartesian, cartesianRotateAttachedIKFrame) {
	const std::string attached_object{ "attached_object" };
	scene->processAttachedCollisionObjectMsg(createAttachedObject(attached_object));
	move->setIKFrame(attached_object);

	move->setDirection([] {
		geometry_msgs::msg::TwistStamped twist;
		twist.header.frame_id = "world";
		twist.twist.angular.z = TAU / 8.0;
		return twist;
	}());

	ASSERT_TRUE(t.plan()) << "Failed to plan";
	EXPECT_CONST_POSITION(move->solutions().front(), attached_object);
}

using PlannerTypes = ::testing::Types<solvers::CartesianPath, solvers::PipelinePlanner>;
TYPED_TEST_SUITE(PandaMoveRelative, PlannerTypes);
TYPED_TEST(PandaMoveRelative, cartesianCollisionMinMaxDistance) {
	if (std::is_same<TypeParam, solvers::PipelinePlanner>::value)
		GTEST_SKIP();  // Pilz PipelinePlanner current fails this test (see #538)

	const std::string object{ "object" };
	geometry_msgs::msg::Pose object_pose;
	object_pose.position.z = 0.4;
	object_pose.orientation.w = 1.0;

	this->scene->processCollisionObjectMsg(createObject(object, object_pose));

	this->move->setIKFrame("panda_hand");
	geometry_msgs::msg::Vector3Stamped v;
	v.header.frame_id = "panda_hand";
	v.vector.z = 0.5;
	this->move->setDirection(v);
	EXPECT_FALSE(this->t.plan()) << "Plan should fail due to collision";

	this->t.reset();
	v.vector.z = 1.0;
	this->move->setDirection(v);
	this->move->setMinMaxDistance(0.1, 0.5);
	EXPECT_TRUE(this->t.plan()) << "Plan should succeed due to min distance reached";
	auto trajectory = std::dynamic_pointer_cast<const SubTrajectory>(this->move->solutions().front());
	EXPECT_TRUE(this->scene->isPathValid(*trajectory->trajectory(), "panda_arm", false));
}

int main(int argc, char** argv) {
	testing::InitGoogleTest(&argc, argv);
	rclcpp::init(argc, argv);

	return RUN_ALL_TESTS();
}
