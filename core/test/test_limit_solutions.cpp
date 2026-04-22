#include <moveit/task_constructor/stages/limit_solutions.h>

#include "stage_mockups.h"

using namespace moveit::task_constructor;

using LimitSolutionsTest = TaskTestBase;

TEST_F(LimitSolutionsTest, returnsMaxSolutions) {
	auto g = std::make_unique<GeneratorMockup>(PredefinedCosts({ 2.0, 1.0, 3.0 }));
	auto ls = std::make_unique<stages::LimitSolutions>("LimitSolutions", std::move(g));
	ls->setMaxSolutions(2);
	t.add(std::move(ls));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.solutions().size(), 2u);
}

TEST_F(LimitSolutionsTest, returnsAllChildSolutionsIfFewerThanMax) {
	auto g = std::make_unique<GeneratorMockup>(PredefinedCosts({ 2.0, 1.0, 3.0 }));
	auto ls = std::make_unique<stages::LimitSolutions>("LimitSolutions", std::move(g));
	ls->setMaxSolutions(5);
	t.add(std::move(ls));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.solutions().size(), 3u);
}

TEST_F(LimitSolutionsTest, returnsBestSolutions) {
	// generator can create solutions with costs 1-4 in a single call to compute
	auto g = std::make_unique<GeneratorMockup>(PredefinedCosts({ 4.0, 2.0, 3.0, 1.0 }), 4);
	auto ls = std::make_unique<stages::LimitSolutions>("LimitSolutions", std::move(g));
	ls->setMaxSolutions(3);  // however, only the first 3 solutions will be considered
	t.add(std::move(ls));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.solutions().size(), 3u);  // only 3 solutions should have been generated
	EXPECT_EQ(t.solutions().front()->cost(), 2.0);  // solution with cost 1.0 was not considered anymore
}

// ensure that the child stage is not called again after max_solutions has been reached
TEST_F(LimitSolutionsTest, childNotCalledAfterLimitReached) {
	auto g = std::make_unique<GeneratorMockup>(PredefinedCosts({ 4.0, 2.0, 3.0, 1.0 }), 1);
	auto* g_ptr = g.get();

	auto ls = std::make_unique<stages::LimitSolutions>("LimitSolutions", std::move(g));
	ls->setMaxSolutions(1);

	auto delayed = std::make_unique<DelayingWrapper>(std::list<unsigned int>{ 0, 0, 1, 1, 1 }, std::move(ls));
	t.add(std::move(delayed));

	EXPECT_TRUE(t.plan());
	EXPECT_EQ(t.solutions().size(), 1u);
	EXPECT_EQ(g_ptr->runs_, 1u);  // generator (child) should run only once
}
