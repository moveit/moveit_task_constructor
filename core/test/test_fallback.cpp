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

struct TestBase : public testing::Test
{
	Task t;
	TestBase() {
		resetMockupIds();
		t.setRobotModel(getModel());
	}
};

using FallbacksFixture = TestBase;

TEST_F(FallbacksFixture, failingNoSolutions) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0), 0));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0), 0));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST_F(FallbacksFixture, failingWithFailedSolutions) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST_F(FallbacksFixture, DISABLED_ConnectStageInsideFallbacks) {
	t.add(std::make_unique<GeneratorMockup>());

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ConnectMockup>());
	t.add(std::move(fallbacks));

	t.add(std::make_unique<GeneratorMockup>());

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST_F(FallbacksFixture, ComputeFirstSuccessfulStageOnly) {
	t.add(std::make_unique<GeneratorMockup>());

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST_F(FallbacksFixture, ActiveChildReset) {
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
