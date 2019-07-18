#include "models.h"

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/console.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;

class GeneratorMockup : public Generator
{
	PlanningScenePtr ps;
	InterfacePtr prev;
	InterfacePtr next;

public:
	GeneratorMockup() : Generator("generator") {
		prev.reset(new Interface);
		next.reset(new Interface);
		pimpl()->setPrevEnds(prev);
		pimpl()->setNextStarts(next);
	}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		ps.reset((new PlanningScene(robot_model)));
	}

	bool canCompute() const override { return true; }
	void compute() override {
		InterfaceState state(ps);
		state.properties().set("target_pose", geometry_msgs::PoseStamped());
		spawn(std::move(state), 0.0);
	}
};

class ConnectMockup : public Connecting
{
public:
	using Connecting::compatible;
	void compute(const InterfaceState& from, const InterfaceState& to) override {}
};

TEST(Stage, registerCallbacks) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);

	GeneratorMockup g;
	g.init(getModel());

	uint called = 0;
	auto cb = [&called](const SolutionBase& s) {
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
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);

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
	props.set("group", std::string("base_from_base_to_tip"));
	EXPECT_NO_THROW(ik.init(robot_model));
}

TEST(ModifyPlanningScene, allowCollisions) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);

	auto s = std::make_unique<stages::ModifyPlanningScene>();
	std::string first = "foo", second = "boom";
	s->allowCollisions(first, second, true);
	s->allowCollisions(first, std::vector<const char*>{ "ab", "abc" }, false);

	s->allowCollisions("foo", std::vector<const char*>{ "bar", "boom" }, true);
	s->allowCollisions("foo", std::set<const char*>{ "ab", "abc" }, false);
}

void spawnObject(PlanningScene& scene, const std::string& name, int type,
                 const std::vector<double>& pos = { 0, 0, 0 }) {
	moveit_msgs::CollisionObject o;
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
		case shape_msgs::SolidPrimitive::CYLINDER:
			o.primitives[0].dimensions = { 0.1, 0.02 };
			break;
		case shape_msgs::SolidPrimitive::BOX:
			o.primitives[0].dimensions = { 0.1, 0.2, 0.3 };
			break;
		case shape_msgs::SolidPrimitive::SPHERE:
			o.primitives[0].dimensions = { 0.05 };
			break;
	}
	scene.processCollisionObjectMsg(o);
}

void attachObject(PlanningScene& scene, const std::string& object, const std::string& link, bool attach) {
	moveit_msgs::AttachedCollisionObject obj;
	obj.link_name = link;
	obj.object.operation =
	    attach ? (int8_t)moveit_msgs::CollisionObject::ADD : (int8_t)moveit_msgs::CollisionObject::REMOVE;
	obj.object.id = object;
	scene.processAttachedCollisionObjectMsg(obj);
}

TEST(Connect, compatible) {
	ConnectMockup connect;
	auto scene = std::make_shared<PlanningScene>(getModel());
	auto& state = scene->getCurrentStateNonConst();
	state.setToDefaultValues();
	spawnObject(*scene, "object", shape_msgs::SolidPrimitive::CYLINDER);
	state.update();

	auto other = scene->diff();
	EXPECT_TRUE(connect.compatible(scene, other)) << "identical scenes";

	spawnObject(*other, "object", shape_msgs::SolidPrimitive::BOX);
	// EXPECT_FALSE(connect.compatible(scene, other)) << "different shapes";

	spawnObject(*other, "object", shape_msgs::SolidPrimitive::CYLINDER, { 0.1, 0, 0 });
	EXPECT_FALSE(connect.compatible(scene, other)) << "different pose";

	spawnObject(*other, "object", shape_msgs::SolidPrimitive::CYLINDER);
	EXPECT_TRUE(connect.compatible(scene, other)) << "same objects";

	// attached objects
	other = scene->diff();
	attachObject(*scene, "object", "base_link", true);
	EXPECT_FALSE(connect.compatible(scene, other)) << "detached and attached object";

	other = scene->diff();
	EXPECT_TRUE(connect.compatible(scene, other)) << "identical scenes, attached object";

	spawnObject(*other, "object", shape_msgs::SolidPrimitive::CYLINDER, { 0.1, 0, 0 });
	attachObject(*other, "object", "base_link", true);
	EXPECT_FALSE(connect.compatible(scene, other)) << "different pose";
}
