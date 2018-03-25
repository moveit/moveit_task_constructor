#include "models.h"

#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/stages/compute_ik.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometry_msgs/PoseStamped.h>

#include <ros/console.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;

class GeneratorMockup : public Generator {
	planning_scene::PlanningScenePtr ps;
	InterfacePtr prev;
	InterfacePtr next;

public:
	GeneratorMockup() : Generator("generator") {
		prev.reset(new Interface);
		next.reset(new Interface);
		pimpl()->setPrevEnds(prev);
		pimpl()->setNextStarts(next);
	}

	void init(const moveit::core::RobotModelConstPtr &robot_model) {
		ps.reset((new planning_scene::PlanningScene(robot_model)));
	}

	bool canCompute() const override { return true; }
	bool compute() override {
		InterfaceState state(ps);
		state.properties().set("target_pose", geometry_msgs::PoseStamped());
		spawn(std::move(state), 0.0);
		return true;
	}
};

TEST(Stage, registerCallbacks) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);

	GeneratorMockup g;
	g.init(getModel());

	uint called = 0;
	auto cb = [&called](const SolutionBase &s) {
		++called;
		return true;
	};

	auto it = g.addSolutionCallback(cb);
	g.compute();
	EXPECT_EQ(called, 1);

	g.removeSolutionCallback(it);
	g.compute();
	EXPECT_EQ(called, 1);
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
