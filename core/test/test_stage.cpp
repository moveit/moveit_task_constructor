#include "models.h"

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "stage_mockups.h"

#include <rclcpp/logging.hpp>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;

struct StandaloneGeneratorMockup : public GeneratorMockup
{
	InterfacePtr prev;
	InterfacePtr next;

	StandaloneGeneratorMockup(std::initializer_list<double>&& costs)
	  : StandaloneGeneratorMockup{ PredefinedCosts{ costs, true } } {}

	StandaloneGeneratorMockup(PredefinedCosts&& costs = PredefinedCosts{ { 0.0 }, true })
	  : GeneratorMockup{ std::move(costs) } {
		prev.reset(new Interface);
		next.reset(new Interface);
		pimpl()->setPrevEnds(prev);
		pimpl()->setNextStarts(next);
	}
};

TEST(Stage, registerCallbacks) {
	StandaloneGeneratorMockup g{ PredefinedCosts::constant(0.0) };
	g.init(getModel());

	uint called = 0;
	auto cb = [&called](const SolutionBase& /* s */) {
		++called;
		return true;
	};

	auto it = g.addSolutionCallback(cb);
	g.compute();
	EXPECT_EQ(called, 1u);

	g.removeSolutionCallback(it);
	g.compute();
	EXPECT_EQ(called, 1u);
}

TEST(ComputeIK, init) {
	auto g = std::make_unique<GeneratorMockup>();
	stages::ComputeIK ik("ik", std::move(g));
	moveit::core::RobotModelPtr robot_model = getModel();
	auto& props = ik.properties();

	// neither eef nor group defined should not throw
	EXPECT_NO_THROW(ik.init(robot_model));

	// invalid eef should throw
	props.set("eef", std::string("undefined"));
	EXPECT_THROW(ik.init(robot_model), InitStageException);

	// valid eef should not throw
	props.set("eef", std::string("eef"));
	EXPECT_NO_THROW(ik.init(robot_model));

	props.set("eef", boost::any());

	// invalid group should throw
	props.set("group", std::string("undefined"));
	EXPECT_THROW(ik.init(robot_model), InitStageException);

	// valid group should not throw
	props.set("group", std::string("group"));
	EXPECT_NO_THROW(ik.init(robot_model));
}

TEST(ModifyPlanningScene, allowCollisions) {
	auto s = std::make_unique<stages::ModifyPlanningScene>();
	std::string first = "foo", second = "boom";
	s->allowCollisions(first, second, true);
	s->allowCollisions(first, std::vector<const char*>{ "ab", "abc" }, false);

	s->allowCollisions("foo", std::vector<const char*>{ "bar", "boom" }, true);
	s->allowCollisions("foo", std::set<const char*>{ "ab", "abc" }, false);
}

void spawnObject(PlanningScene& scene, const std::string& name, int type,
                 const std::vector<double>& pos = { 0, 0, 0 }) {
	moveit_msgs::msg::CollisionObject o;
	o.id = name;
	o.header.frame_id = scene.getPlanningFrame();
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = pos[0];
	o.primitive_poses[0].position.y = pos[1];
	o.primitive_poses[0].position.z = pos[2];
	o.primitive_poses[0].orientation.w = 1.0;

	o.primitives.resize(1);
	o.primitives[0].type = type;

	switch (type) {
		case shape_msgs::msg::SolidPrimitive::CYLINDER:
			o.primitives[0].dimensions = { 0.1, 0.02 };
			break;
		case shape_msgs::msg::SolidPrimitive::BOX:
			o.primitives[0].dimensions = { 0.1, 0.2, 0.3 };
			break;
		case shape_msgs::msg::SolidPrimitive::SPHERE:
			o.primitives[0].dimensions = { 0.05 };
			break;
	}
	scene.processCollisionObjectMsg(o);
}

void attachObject(PlanningScene& scene, const std::string& object, const std::string& link, bool attach) {
	moveit_msgs::msg::AttachedCollisionObject obj;
	obj.link_name = link;
	obj.object.operation = attach ? static_cast<int8_t>(moveit_msgs::msg::CollisionObject::ADD) :
	                                static_cast<int8_t>(moveit_msgs::msg::CollisionObject::REMOVE);
	obj.object.id = object;
	scene.processAttachedCollisionObjectMsg(obj);
}

TEST(Connect, compatible) {
	ConnectMockup connect;
	auto scene = std::make_shared<PlanningScene>(getModel());
	auto& state = scene->getCurrentStateNonConst();
	state.setToDefaultValues();
	spawnObject(*scene, "object", shape_msgs::msg::SolidPrimitive::CYLINDER);
	state.update();

	auto other = scene->diff();
	EXPECT_TRUE(connect.compatible(scene, other)) << "identical scenes";

	spawnObject(*other, "object", shape_msgs::msg::SolidPrimitive::BOX);
	// EXPECT_FALSE(connect.compatible(scene, other)) << "different shapes";

	spawnObject(*other, "object", shape_msgs::msg::SolidPrimitive::CYLINDER, { 0.1, 0, 0 });
	EXPECT_FALSE(connect.compatible(scene, other)) << "different pose";

	spawnObject(*other, "object", shape_msgs::msg::SolidPrimitive::CYLINDER);
	EXPECT_TRUE(connect.compatible(scene, other)) << "same objects";

	// attached objects
	other = scene->diff();
	attachObject(*scene, "object", "tip", true);
	EXPECT_FALSE(connect.compatible(scene, other)) << "detached and attached object";

	other = scene->diff();
	EXPECT_TRUE(connect.compatible(scene, other)) << "identical scenes, attached object";

	spawnObject(*other, "object", shape_msgs::msg::SolidPrimitive::CYLINDER, { 0.1, 0, 0 });
	attachObject(*other, "object", "tip", true);
	EXPECT_FALSE(connect.compatible(scene, other)) << "different pose";
}
