#include <moveit/task_constructor/task.h>

#include "stage_mockups.h"
#include "models.h"

#include <list>
#include <memory>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;

using Pruning = TaskTestBase;

TEST_F(Pruning, PropagatorFailure) {
	auto back = add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));
	add(t, new ForwardMockup({ INF }));

	EXPECT_FALSE(t.plan());
	ASSERT_EQ(t.solutions().size(), 0u);
	// ForwardMockup fails, so the backward stage should never compute
	EXPECT_EQ(back->runs_, 0u);
}

TEST_F(Pruning, PruningMultiForward) {
	add(t, new BackwardMockup());
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup());
	// spawn two solutions for the only incoming state
	add(t, new ForwardMockup(PredefinedCosts{ { 0.0, 0.0 } }, 2));
	// fail to extend the second solution
	add(t, new ForwardMockup({ 0, INF }));

	EXPECT_TRUE(t.plan());

	// the second (infeasible) solution in the last stage must not disable
	// the earlier partial solution just because they share stage solutions
	ASSERT_EQ(t.solutions().size(), 1u);
	EXPECT_EQ((*t.solutions().begin())->cost(), 0u);
}

TEST_F(Pruning, ConnectConnectForward) {
	add(t, new GeneratorMockup());
	auto c1 = add(t, new ConnectMockup({ INF, 0, 0 }));  // 1st attempt is a failue
	add(t, new GeneratorMockup({ 0, 10, 20 }));
	add(t, new ForwardMockup());
	auto c2 = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 1, 2, 3 }));

	t.plan();

	ASSERT_EQ(t.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : t.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->runs_, 3u);
	EXPECT_EQ(c2->runs_, 6u);  // expect 6 instead of 9 calls
}

TEST_F(Pruning, ConnectConnectBackward) {
	add(t, new GeneratorMockup({ 1, 2, 3 }));
	auto c1 = add(t, new ConnectMockup());
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0, INF, 10, 20 }));  // 2nd is a dummy to postpone creation of 3rd
	auto c2 = add(t, new ConnectMockup({ INF, 0, 0, 0 }));  // 1st attempt is a failure
	add(t, new GeneratorMockup());

	t.plan();

	ASSERT_EQ(t.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : t.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->runs_, 6u);  // expect 6 instead of 9 calls
	EXPECT_EQ(c2->runs_, 3u);
}

TEST_F(Pruning, PropagateIntoContainer) {
	add(t, new BackwardMockup({ INF }));
	add(t, new GeneratorMockup({ 0 }));

	auto inner = add(t, new SerialContainer());
	auto con = add(*inner, new ConnectMockup());
	add(*inner, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(t.plan());

	// the failure in the backward stage (outside the container)
	// should prune the expected computation of con inside the container
	EXPECT_EQ(con->runs_, 0u);
}

TEST_F(Pruning, PropagateFromContainerPull) {
	auto back = add(t, new BackwardMockup());
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));

	auto inner = add(t, new SerialContainer());
	add(*inner, new ForwardMockup());
	add(*inner, new ForwardMockup({ INF }));

	EXPECT_FALSE(t.plan());

	// the failure inside the container should prune computing of back
	EXPECT_EQ(back->runs_, 0u);
}

TEST_F(Pruning, PropagateFromContainerPush) {
	auto inner = add(t, new SerialContainer());
	add(*inner, new BackwardMockup({ INF }));

	add(t, new GeneratorMockup({ 0 }));
	auto con = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(t.plan());

	// the failure inside container should prune computing of con
	EXPECT_EQ(con->runs_, 0u);
}

TEST_F(Pruning, PropagateFromParallelContainerMultiplePaths) {
	auto back = add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));
	auto inner = add(t, new Alternatives());

	add(*inner, new ForwardMockup({ INF }));
	auto serial = add(*inner, new SerialContainer());
	add(*serial, new ConnectMockup());
	add(*serial, new GeneratorMockup({ 0 }));

	EXPECT_TRUE(t.plan());

	// the failure in one branch of Alternatives must not prune computing back
	EXPECT_EQ(back->runs_, 1u);
}
