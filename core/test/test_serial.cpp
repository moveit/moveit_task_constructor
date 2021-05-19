#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/robot_model_test_utils.h>

#include "models.h"
#include <list>
#include <memory>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;

MOVEIT_STRUCT_FORWARD(PredefinedCosts);
struct PredefinedCosts : CostTerm
{
	mutable std::list<double> costs_;  // list of costs to assign
	mutable double cost_ = 0.0;  // last assigned cost
	bool finite_;  // finite number of compute() attempts?

	PredefinedCosts(bool finite, std::list<double>&& costs) : costs_(std::move(costs)), finite_(finite) {}
	bool exhausted() const { return finite_ && costs_.empty(); }
	double cost() const {
		if (!costs_.empty()) {
			cost_ = costs_.front();
			costs_.pop_front();
		}
		return cost_;
	}

	double operator()(const SubTrajectory& s, std::string& comment) const override { return cost(); }
	double operator()(const SolutionSequence& s, std::string& comment) const override { return cost(); }
	double operator()(const WrappedSolution& s, std::string& comment) const override { return cost(); }
};

/** Generator creating solutions with given costs */
struct GeneratorMockup : Generator
{
	PlanningScenePtr ps_;
	PredefinedCosts costs_;
	static unsigned int id_;

	GeneratorMockup(std::initializer_list<double> costs = { 0.0 })
	  : Generator("GEN" + std::to_string(++id_)), costs_(true, costs) {}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		ps_.reset(new PlanningScene(robot_model));
		Generator::init(robot_model);
	}

	bool canCompute() const override { return !costs_.exhausted(); }
	void compute() override { spawn(InterfaceState(ps_), costs_.cost()); }
};

struct PropagatorMockup : public PropagatingEitherWay
{
	PredefinedCosts costs_;
	std::size_t solutions_per_compute_;

	unsigned int calls_ = 0;

	PropagatorMockup(std::initializer_list<double> costs = { 0.0 }, std::size_t solutions_per_compute = 1)
	  : PropagatingEitherWay(), costs_(false, costs), solutions_per_compute_(solutions_per_compute) {}

	void computeForward(const InterfaceState& from) override {
		++calls_;
		for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
			SubTrajectory solution(robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost());
			sendForward(from, InterfaceState(from.scene()->diff()), std::move(solution));
		}
	}
	void computeBackward(const InterfaceState& to) override {
		++calls_;
		for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
			SubTrajectory solution(robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost());
			sendBackward(InterfaceState(to.scene()->diff()), to, std::move(solution));
		}
	}
};
struct ForwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	ForwardMockup(std::initializer_list<double> costs = { 0.0 }, std::size_t solutions_per_compute = 1)
	  : PropagatorMockup(costs, solutions_per_compute) {
		restrictDirection(FORWARD);
		setName("FW" + std::to_string(++id_));
	}
};
struct BackwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	BackwardMockup(std::initializer_list<double> costs = { 0.0 }) : PropagatorMockup(costs) {
		restrictDirection(BACKWARD);
		setName("BW" + std::to_string(++id_));
	}
};

/* Forward propagator, contributing no solutions at all */
struct ForwardDummy : PropagatingForward
{
	using PropagatingForward::PropagatingForward;
	void computeForward(const InterfaceState& /*from*/) override {}
};

/* Connect creating solutions with given costs */
struct Connect : stages::Connect
{
	PlanningScenePtr ps_;
	PredefinedCostsPtr costs_;
	unsigned int calls_ = 0;
	static unsigned int id_;

	static GroupPlannerVector getPlanners() {
		auto planner = std::make_shared<solvers::JointInterpolationPlanner>();
		return { { "group", planner }, { "eef_group", planner } };
	}

	Connect(std::initializer_list<double> costs = {}, bool enforce_sequential = false)
	  : stages::Connect("CON" + std::to_string(++id_), getPlanners()) {
		costs_ = std::make_shared<PredefinedCosts>(false, costs);
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
unsigned int GeneratorMockup::id_ = 0;
unsigned int ForwardMockup::id_ = 0;
unsigned int BackwardMockup::id_ = 0;
unsigned int Connect::id_ = 0;

struct TestBase : public testing::Test
{
	Task task;
	TestBase() {
		resetIds();
		task.setRobotModel(getModel());
	}

	void resetIds() {
		GeneratorMockup::id_ = 0;
		ForwardMockup::id_ = 0;
		BackwardMockup::id_ = 0;
		Connect::id_ = 0;
	}
	template <typename C, typename S>
	auto add(C& container, S* stage) -> S* {
		container.add(Stage::pointer(stage));
		return stage;
	}
};

using ConnectConnect = TestBase;
// https://github.com/ros-planning/moveit_task_constructor/issues/182
TEST_F(ConnectConnect, SuccSucc) {
	add(task, new GeneratorMockup({ 1, 2, 3 }));
	add(task, new Connect());
	add(task, new GeneratorMockup({ 10, 20 }));
	add(task, new Connect());
	add(task, new GeneratorMockup());

	EXPECT_TRUE(task.plan());
	ASSERT_EQ(task.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : task.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
}

// https://github.com/ros-planning/moveit_task_constructor/issues/218
TEST_F(ConnectConnect, FailSucc) {
	add(task, new GeneratorMockup());
	add(task, new Connect({ INF }, true));
	add(task, new GeneratorMockup());
	add(task, new Connect());
	add(task, new GeneratorMockup());
	add(task, new ForwardDummy());

	EXPECT_FALSE(task.plan());
}

using Pruning = TestBase;
TEST_F(Pruning, PropagatorFailure) {
	auto back = add(task, new BackwardMockup());
	add(task, new GeneratorMockup({ 0 }));
	add(task, new ForwardMockup({ INF }));

	EXPECT_FALSE(task.plan());
	ASSERT_EQ(task.solutions().size(), 0);
	// ForwardMockup fails, so the backward stage should never compute
	EXPECT_EQ(back->calls_, 0);
}

TEST_F(Pruning, PruningMultiForward) {
	add(task, new BackwardMockup());
	add(task, new BackwardMockup());
	add(task, new GeneratorMockup());
	// spawn two solutions for the only incoming state
	add(task, new ForwardMockup({ 0, 0 }, 2));
	// fail to extend the second solution
	add(task, new ForwardMockup({ 0, INF }));

	EXPECT_TRUE(task.plan());

	// the second (infeasible) solution in the last stage must not disable
	// the earlier partial solution just because they share stage solutions
	ASSERT_EQ(task.solutions().size(), 1);
	EXPECT_EQ((*task.solutions().begin())->cost(), 0u);
}

TEST_F(Pruning, ConnectConnectForward) {
	add(task, new GeneratorMockup());
	auto c1 = add(task, new Connect({ INF, 0 }));  // 1st attempt is a failue
	add(task, new GeneratorMockup({ 0, 10, 20 }));
	add(task, new ForwardMockup());
	auto c2 = add(task, new Connect());
	add(task, new GeneratorMockup({ 1, 2, 3 }));

	task.plan();

	ASSERT_EQ(task.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : task.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->calls_, 3u);
	EXPECT_EQ(c2->calls_, 6u);  // expect 6 instead of 9 calls
}

TEST_F(Pruning, ConnectConnectBackward) {
	add(task, new GeneratorMockup({ 1, 2, 3 }));
	auto c1 = add(task, new Connect());
	add(task, new BackwardMockup());
	add(task, new GeneratorMockup({ 0, INF, 10, 20 }));  // 2nd is a dummy to postpone creation of 3rd
	auto c2 = add(task, new Connect({ INF, 0 }));  // 1st attempt is a failure
	add(task, new GeneratorMockup());

	task.plan();

	ASSERT_EQ(task.solutions().size(), 3u * 2u);
	std::vector<double> expected_costs = { 11, 12, 13, 21, 22, 23 };
	auto expected_cost = expected_costs.begin();
	for (const auto& s : task.solutions()) {
		EXPECT_EQ(s->cost(), *expected_cost);
		++expected_cost;
	}
	EXPECT_EQ(c1->calls_, 6u);  // expect 6 instead of 9 calls
	EXPECT_EQ(c2->calls_, 3u);
}

TEST_F(Pruning, PropagateIntoContainer) {
	add(task, new BackwardMockup({ INF }));
	add(task, new GeneratorMockup({ 0 }));

	auto inner = add(task, new SerialContainer());
	auto con = add(*inner, new Connect());
	add(*inner, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(task.plan());

	// the failure in the backward stage (outside the container)
	// should prune the expected computation of con inside the container
	EXPECT_EQ(con->calls_, 0);
}

TEST_F(Pruning, PropagateFromContainerPull) {
	auto back = add(task, new BackwardMockup());
	add(task, new BackwardMockup());
	add(task, new GeneratorMockup({ 0 }));

	auto inner = add(task, new SerialContainer());
	add(*inner, new ForwardMockup());
	add(*inner, new ForwardMockup({ INF }));

	EXPECT_FALSE(task.plan());

	// the failure inside the container should prune computing of back
	EXPECT_EQ(back->calls_, 0);
}

TEST_F(Pruning, PropagateFromContainerPush) {
	auto inner = add(task, new SerialContainer());
	add(*inner, new BackwardMockup({ INF }));

	add(task, new GeneratorMockup({ 0 }));
	auto con = add(task, new Connect());
	add(task, new GeneratorMockup({ 0 }));

	EXPECT_FALSE(task.plan());

	// the failure inside container should prune computing of con
	EXPECT_EQ(con->calls_, 0);
}

TEST_F(Pruning, PropagateFromParallelContainerMultiplePaths) {
	auto back = add(task, new BackwardMockup());
	add(task, new GeneratorMockup({ 0 }));
	auto inner = add(task, new Alternatives());

	add(*inner, new ForwardMockup({ INF }));
	auto serial = add(*inner, new SerialContainer());
	add(*serial, new Connect());
	add(*serial, new GeneratorMockup({ 0 }));

	EXPECT_TRUE(task.plan());

	// the failure in one branch of Alternatives must not prune computing back
	EXPECT_EQ(back->calls_, 1);
}
