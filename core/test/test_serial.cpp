#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/robot_model_test_utils.h>

#include "stage_mockups.h"
#include "models.h"
#include <list>
#include <memory>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;

/* Connect creating solutions with given costs */
struct Connect : stages::Connect
{
	PredefinedCostsPtr costs_;
	unsigned int calls_{ 0 };
	static unsigned int id_;

	static GroupPlannerVector getPlanners() {
		auto planner = std::make_shared<solvers::JointInterpolationPlanner>();
		return { { "group", planner }, { "eef_group", planner } };
	}

	Connect(std::initializer_list<double> costs = { 0.0 }, bool enforce_sequential = false)
	  : stages::Connect("CON" + std::to_string(++id_), getPlanners()) {
		costs_ = std::make_shared<PredefinedCosts>(costs, false);
		setCostTerm(costs_);
		if (enforce_sequential)
			setProperty("merge_mode", SEQUENTIAL);
	}
	void compute(const InterfaceState& from, const InterfaceState& to) override {
		++calls_;
		stages::Connect::compute(from, to);
	}
};

constexpr double INF = std::numeric_limits<double>::infinity();
unsigned int Connect::id_ = 0;

struct TestBase : public TaskTestBase
{
	TestBase() { Connect::id_ = 0; }
};

using ConnectConnect = TestBase;
// https://github.com/ros-planning/moveit_task_constructor/issues/182
TEST_F(ConnectConnect, SuccSucc) {
	add(t, new GeneratorMockup({ 1.0, 2.0, 3.0 }));
	add(t, new Connect());
	add(t, new GeneratorMockup({ 10.0, 20.0 }));
	add(t, new Connect());
	add(t, new GeneratorMockup({ 0.0 }));

	EXPECT_TRUE(t.plan());
	ASSERT_EQ(t.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : t.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
}

// https://github.com/ros-planning/moveit_task_constructor/issues/218
TEST_F(ConnectConnect, FailSucc) {
	add(t, new GeneratorMockup());
	add(t, new Connect({ INF }, true));
	add(t, new GeneratorMockup());
	add(t, new Connect());
	add(t, new GeneratorMockup());
	add(t, new ForwardMockup(PredefinedCosts::constant(0.0), 0));

	EXPECT_FALSE(t.plan());
}

using Pruning = TestBase;
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
	auto c1 = add(t, new Connect({ INF, 0 }));  // 1st attempt is a failue
	add(t, new GeneratorMockup({ 0, 10, 20 }));
	add(t, new ForwardMockup());
	auto c2 = add(t, new Connect());
	add(t, new GeneratorMockup({ 1, 2, 3 }));

	t.plan();

	ASSERT_EQ(t.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : t.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->calls_, 3u);
	EXPECT_EQ(c2->calls_, 6u);  // expect 6 instead of 9 calls
}

TEST_F(Pruning, ConnectConnectBackward) {
	add(t, new GeneratorMockup({ 1, 2, 3 }));
	auto c1 = add(t, new Connect());
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0, INF, 10, 20 }));  // 2nd is a dummy to postpone creation of 3rd
	auto c2 = add(t, new Connect({ INF, 0 }));  // 1st attempt is a failure
	add(t, new GeneratorMockup());

	t.plan();

	ASSERT_EQ(t.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : t.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->calls_, 6u);  // expect 6 instead of 9 calls
	EXPECT_EQ(c2->calls_, 3u);
}

TEST_F(Pruning, PropagateIntoContainer) {
	add(t, new BackwardMockup({ INF }));
	add(t, new GeneratorMockup({ 0 }));

	auto inner = add(t, new SerialContainer());
	auto con = add(*inner, new Connect());
	add(*inner, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(t.plan());

	// the failure in the backward stage (outside the container)
	// should prune the expected computation of con inside the container
	EXPECT_EQ(con->calls_, 0u);
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
	auto con = add(t, new Connect());
	add(t, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(t.plan());

	// the failure inside container should prune computing of con
	EXPECT_EQ(con->calls_, 0u);
}

TEST_F(Pruning, PropagateFromParallelContainerMultiplePaths) {
	auto back = add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));
	auto inner = add(t, new Alternatives());

	add(*inner, new ForwardMockup({ INF }));
	auto serial = add(*inner, new SerialContainer());
	add(*serial, new Connect());
	add(*serial, new GeneratorMockup({ 0 }));

	EXPECT_TRUE(t.plan());

	// the failure in one branch of Alternatives must not prune computing back
	EXPECT_EQ(back->runs_, 1u);
}
