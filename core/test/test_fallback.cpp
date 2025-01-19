#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/stages/predicate_filter.h>
#include <moveit/task_constructor/stages/noop.h>
#include <moveit/planning_scene/planning_scene.h>

#include "stage_mockups.h"
#include "models.h"
#include "gtest_value_printers.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <initializer_list>
#include <chrono>
#include <thread>

using namespace moveit::task_constructor;

using FallbacksFixtureGenerator = TaskTestBase;

TEST_F(FallbacksFixtureGenerator, stayWithFirstSuccessful) {
	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(INF)));
	fallback->add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(1.0)));
	fallback->add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(2.0)));
	t.add(std::move(fallback));

	EXPECT_TRUE(t.plan());
	ASSERT_EQ(t.solutions().size(), 1u);
	EXPECT_EQ(t.solutions().front()->cost(), 1.0);
}

using FallbacksFixturePropagate = TaskTestBase;

TEST_F(FallbacksFixturePropagate, failingNoSolutions) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts({}), 0));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts({}), 0));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST_F(FallbacksFixturePropagate, failingWithFailedSolutions) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(INF)));
	t.add(std::move(fallback));

	EXPECT_FALSE(t.plan());
	EXPECT_EQ(t.solutions().size(), 0u);
}

TEST_F(FallbacksFixturePropagate, computeFirstSuccessfulStageOnly) {
	t.add(std::make_unique<GeneratorMockup>());

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.numSolutions(), 1u);
}

TEST_F(FallbacksFixturePropagate, computeFirstSuccessfulStagePerSolutionOnly) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 2.0, 1.0 })));
	// duplicate generator solutions with resulting costs: 4, 2 | 3, 1
	t.add(std::make_unique<ForwardMockup>(PredefinedCosts({ 2.0, 0.0, 2.0, 0.0 }), 2));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF, INF, 110.0, 120.0 })));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts({ 210.0, 220.0, 0, 0 })));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(113, 124, 212, 221));
}

// requires individual job control in Fallbacks's children
TEST_F(FallbacksFixturePropagate, DISABLED_updateSolutionOrder) {
	t.add(std::make_unique<BackwardMockup>(PredefinedCosts({ 10.0, 0.0 })));
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 1.0, 2.0 })));
	// available solutions (sorted) in individual runs of fallbacks: 1 | 11, 2 | 2, 11

	// use a fallback container to delay computation twice: only the last child succeeds
	auto inner = std::make_unique<Fallbacks>("Inner");
	inner->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF }, false)));
	inner->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF }, false)));
	inner->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(0.0)));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::move(inner));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan(1));  // only return 1st solution
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(2));  // expecting less costly solution as result
}

// requires individual job control in Fallbacks's children
TEST_F(FallbacksFixturePropagate, DISABLED_multipleActivePendingStates) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 2.0, 1.0, 3.0 })));
	// use a fallback container to delay computation: the 1st child never succeeds, but only the 2nd
	auto inner = std::make_unique<Fallbacks>("Inner");
	inner->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF }, false)));  // always fail
	inner->add(std::make_unique<ForwardMockup>(PredefinedCosts({ 10.0, INF, 30.0 })));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::move(inner));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF })));
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(11, 33));
	// check that first solution is not marked as pruned
}

TEST_F(FallbacksFixturePropagate, successfulWithMixedSolutions) {
	t.add(std::make_unique<GeneratorMockup>());

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts({ INF, 1.0 }), 2));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::single(2.0)));
	t.add(std::move(fallback));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(1.0));
}

TEST_F(FallbacksFixturePropagate, successfulWithMixedSolutions2) {
	t.add(std::make_unique<GeneratorMockup>());

	auto fallback = std::make_unique<Fallbacks>("Fallbacks");
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts({ 1.0, INF }), 2));
	fallback->add(std::make_unique<ForwardMockup>(PredefinedCosts::single(2.0)));
	t.add(std::move(fallback));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(1.0));
}

TEST_F(FallbacksFixturePropagate, activeChildReset) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 1.0, INF, 3.0 })));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(10.0)));
	fallbacks->add(std::make_unique<ForwardMockup>(PredefinedCosts::constant(20.0)));
	auto fwd1 = fallbacks->findChild("FWD1");
	auto fwd2 = fallbacks->findChild("FWD2");
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(11, 13));
	EXPECT_COSTS(fwd1->solutions(), testing::ElementsAre(10, 10));
	EXPECT_COSTS(fwd2->solutions(), testing::IsEmpty());
}

// https://github.com/moveit/moveit_task_constructor/issues/581#issuecomment-2147985474
TEST_F(FallbacksFixturePropagate, filterPropagatesFailures) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts::single(0.0)));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	auto add_filtered_fwd = [&fallbacks](double cost, bool accept) {
		auto fwd = std::make_unique<ForwardMockup>(PredefinedCosts::constant(cost));
		auto filter = std::make_unique<stages::PredicateFilter>("filter", std::move(fwd));
		filter->setPredicate([accept](const SolutionBase& /*solution*/, std::string& /*comment*/) { return accept; });
		fallbacks->add(std::move(filter));
	};
	add_filtered_fwd(INF, false);  // Propagate fails, filter rejects
	add_filtered_fwd(2.0, true);  // Propagate succeeds, filter accepts
	fallbacks->add(std::make_unique<stages::NoOp>());
	t.add(std::move(fallbacks));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(2.));
}

using FallbacksFixtureConnect = TaskTestBase;

TEST_F(FallbacksFixtureConnect, connectStageInsideFallbacks) {
	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 1.0, 2.0 })));

	auto fallbacks = std::make_unique<Fallbacks>("Fallbacks");
	fallbacks->add(std::make_unique<ConnectMockup>(PredefinedCosts({ 0.0, 0.0, INF, 0.0 })));
	fallbacks->add(std::make_unique<ConnectMockup>(PredefinedCosts::constant(100.0)));
	t.add(std::move(fallbacks));

	t.add(std::make_unique<GeneratorMockup>(PredefinedCosts({ 10.0, 20.0 })));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), testing::ElementsAre(11, 12, 22, 121));
}
