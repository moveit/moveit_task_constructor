#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/planning_scene/planning_scene.h>

#include "stage_mockups.h"
#include "models.h"
#include "gtest_value_printers.h"

#include <gtest/gtest.h>
#include <initializer_list>
#include <chrono>
#include <thread>

using namespace moveit::task_constructor;

constexpr double INF = std::numeric_limits<double>::infinity();

TEST(Fallback, failingNoSolutions) {
	resetMockupIds();
	Task t;
	t.setRobotModel(getModel());

	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0), 0));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0), 0));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST(Fallback, failingWithFailedSolutions) {
	resetMockupIds();
	Task t;
	t.setRobotModel(getModel());

	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST(Fallback, DISABLED_ConnectStageInsideFallbacks) {
	resetMockupIds();
	Task t;
	t.setRobotModel(getModel());

	t.add(std::make_unique<GeneratorMockup>());

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ConnectMockup>());
	t.add(std::move(fallbacks));

	t.add(std::make_unique<GeneratorMockup>());

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST(Fallback, ComputeFirstSuccessfulStageOnly) {
	resetMockupIds();
	Task t;
	t.setRobotModel(getModel());

	t.add(std::make_unique<GeneratorMockup>());

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST(Fallback, ActiveChildReset) {
	resetMockupIds();
	Task t;
	t.setRobotModel(getModel());

	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts{ { 0.0, INF, 0.0 } }));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	auto first = fallbacks->findChild("FWD1");
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 2u);
	EXPECT_EQ(first->solutions().size(), 2u);
}
