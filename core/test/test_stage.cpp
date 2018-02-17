#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/init.h>

#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

class GeneratorMockup : public Generator {
	planning_scene::PlanningScenePtr ps;
	InterfacePtr prev;
	InterfacePtr next;

public:
	GeneratorMockup() : Generator("generator") {
		robot_model_loader::RobotModelLoader loader;
		ps.reset(new planning_scene::PlanningScene(loader.getModel()));

		prev.reset(new Interface);
		next.reset(new Interface);
		pimpl()->setPrevEnds(prev);
		pimpl()->setNextStarts(next);
	}
	bool canCompute() const override { return true; }
	bool compute() override {
		spawn(InterfaceState(ps), 0.0);
		return true;
	}
};

TEST(Stage, registerCallbacks) {
	int argc = 0;
	char *argv = nullptr;
	ros::init(argc, &argv, "testLocalTaskModel");

	GeneratorMockup g;
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
