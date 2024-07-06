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

unsigned int Connect::id_ = 0;

struct TestBase : public TaskTestBase
{
	TestBase() { Connect::id_ = 0; }
};

using ConnectConnect = TestBase;
// https://github.com/moveit/moveit_task_constructor/issues/182
TEST_F(ConnectConnect, SuccSucc) {
	add(t, new GeneratorMockup({ 1.0, 2.0, 3.0 }));
	add(t, new Connect());
	add(t, new GeneratorMockup({ 10.0, 20.0 }));
	add(t, new Connect());
	add(t, new GeneratorMockup({ 0.0 }));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ::testing::ElementsAre(11, 12, 13, 21, 22, 23));
}

// https://github.com/moveit/moveit_task_constructor/issues/218
TEST_F(ConnectConnect, FailSucc) {
	add(t, new GeneratorMockup());
	add(t, new Connect({ INF }, true));
	add(t, new GeneratorMockup());
	add(t, new Connect());
	add(t, new GeneratorMockup());
	add(t, new ForwardMockup(PredefinedCosts::constant(0.0), 0));

	EXPECT_FALSE(t.plan());
}

// https://github.com/moveit/moveit_task_constructor/issues/485#issuecomment-1760606116
TEST_F(ConnectConnect, UniqueEnumeration) {
	add(t, new GeneratorMockup({ 1.0, 2.0, 3.0 }));
	auto con1 = add(t, new ConnectMockup());
	add(t, new GeneratorMockup({ 10.0, 20.0 }));
	auto con2 = add(t, new ConnectMockup({ INF, 0.0, 0.0, 0.0 }));
	add(t, new GeneratorMockup({ 100.0, 200.0 }));

	EXPECT_TRUE(t.plan());
	EXPECT_COSTS(t.solutions(), ::testing::ElementsAre(121, 122, 123, 211, 212, 213, 221, 222, 223));
	EXPECT_EQ(con1->runs_, 3u * 2u);
	EXPECT_EQ(con2->runs_, 2u * 2u);
}
