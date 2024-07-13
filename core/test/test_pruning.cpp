#include <moveit/task_constructor/task.h>

#include "stage_mockups.h"
#include "models.h"

#include <list>
#include <memory>

using namespace moveit::task_constructor;

struct Pruning : TaskTestBase
{
	Pruning() : TaskTestBase() { t.setPruning(true); }
};

TEST_F(Pruning, PropagatorFailure) {
	auto back = add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));
	add(t, new ForwardMockup({ INF }));

	EXPECT_FALSE(t.plan());
	ASSERT_EQ(t.solutions().size(), 0u);
	// ForwardMockup fails, so the backward stage should never compute
	EXPECT_EQ(back->runs_, 0u);
}

// Same as the previous test, except pruning is disabled for the whole task
TEST_F(Pruning, DisabledPruningPropagatorFailure) {
	t.setPruning(false);
	auto back = add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0 }));
	add(t, new ForwardMockup({ INF }));
	EXPECT_FALSE(t.plan());
	ASSERT_EQ(t.solutions().size(), 0u);
	// ForwardMockup fails, since we have pruning disabled backward should run
	EXPECT_EQ(back->runs_, 1u);
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

// The 2nd failing FW attempt would prune the path through CON,
// but shouldn't because there exist two more GEN2 solutions
TEST_F(Pruning, NoPruningIfAlternativesExist) {
	add(t, new GeneratorMockup(PredefinedCosts({ 0.0 })));
	add(t, new ConnectMockup());
	add(t, new GeneratorMockup(std::list<double>{ 0, 10, 20, 30 }, 2));
	add(t, new ForwardMockup({ INF, INF, 0.0, INF }));

	t.plan();

	EXPECT_EQ(t.solutions().size(), 1u);
}

TEST_F(Pruning, ConnectReactivatesPrunedPaths) {
	add(t, new BackwardMockup);
	add(t, new GeneratorMockup({ 0 }));
	add(t, new ConnectMockup());
	// the solution here should re-activate the initially pruned backward path
	add(t, new GeneratorMockup({ 0 }));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.solutions().size(), 1u);
}

// same as before, but wrapping Connect into a container
template <typename T>
struct PruningContainerTests : public Pruning
{};

using ContainerTypes = ::testing::Types<SerialContainer, Fallbacks>;
TYPED_TEST_SUITE(PruningContainerTests, ContainerTypes);
TYPED_TEST(PruningContainerTests, ConnectReactivatesPrunedPaths) {
	this->add(this->t, new BackwardMockup);
	this->add(this->t, new GeneratorMockup({ 0 }));
	auto c = new TypeParam();
	this->add(*c, new ConnectMockup());
	this->add(this->t, c);
	this->add(this->t, new GeneratorMockup({ 0 }));

	EXPECT_TRUE(this->t.plan());
	EXPECT_EQ(this->t.solutions().size(), 1u);
}

TEST_F(Pruning, ConnectConnectForward) {
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup());
	auto c1 = add(t, new ConnectMockup({ INF, 0, 0 }));  // 1st attempt is a failue
	add(t, new GeneratorMockup({ 0, 10, 20 }));
	add(t, new ForwardMockup());
	auto c2 = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 1, 2, 3 }));

	t.plan();
	EXPECT_COSTS(t.solutions(), ::testing::ElementsAre(11, 12, 13, 21, 22, 23));
	EXPECT_EQ(c1->runs_, 3u);
	EXPECT_EQ(c2->runs_, 6u);  // expect 6 instead of 9 calls
}

TEST_F(Pruning, ConnectConnectBackward) {
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 1, 2, 3 }));
	auto c1 = add(t, new ConnectMockup());
	add(t, new BackwardMockup());
	add(t, new GeneratorMockup({ 0, INF, 10, 20 }));  // 2nd is a dummy to postpone creation of 3rd
	auto c2 = add(t, new ConnectMockup({ INF, 0, 0, 0 }));  // 1st attempt is a failure
	add(t, new GeneratorMockup());

	t.plan();

	EXPECT_COSTS(t.solutions(), ::testing::ElementsAre(11, 12, 13, 21, 22, 23));
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

TEST_F(Pruning, DISABLED_PropagateIntoContainerAndReactivate) {
	add(t, new GeneratorMockup({ 0 }));

	auto serial = add(t, new SerialContainer());
	auto con = add(*serial, new ConnectMockup({ 10, 20 }));
	add(*serial, new GeneratorMockup({ 0, 1 }));

	add(t, new ForwardMockup({ INF, 0 }));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(con->runs_, 1u);
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

TEST_F(Pruning, TwoConnects) {
	add(t, new GeneratorMockup({ 0 }));
	add(t, new ForwardMockup({ INF }));
	add(t, new ConnectMockup());

	add(t, new GeneratorMockup());
	add(t, new ConnectMockup());

	add(t, new GeneratorMockup());
	add(t, new ForwardMockup());

	EXPECT_FALSE(t.plan());
}

TEST_F(Pruning, BackPropagateFailure) {
	add(t, new GeneratorMockup({ 1.0 }));
	auto con1 = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 10.0, 20.0 }, 2));  // create all solutions on first run
	auto con2 = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 100.0, 200.0 }, 2));  // create all solutions on first run
	// delay failure (INF) until CON2 has found first solution
	add(t, new DelayingWrapper({ 1 }, std::make_unique<ForwardMockup>(PredefinedCosts({ INF, 2000 }))));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ::testing::ElementsAre(2211, 2221));
	EXPECT_EQ(con1->runs_, 2u);
	EXPECT_EQ(con2->runs_, 3u);  // 100 - 20 is pruned
}
