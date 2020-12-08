#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>

#include "models.h"
#include <memory>
#include <algorithm>
#include <iterator>
#include <vector>
#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

using namespace moveit::task_constructor;
TEST(InterfaceStatePriority, compare) {
	using Prio = InterfaceState::Priority;
	constexpr double inf = std::numeric_limits<double>::infinity();

	EXPECT_TRUE(Prio(0, 0) == Prio(0, 0));
	EXPECT_TRUE(Prio(0, inf) == Prio(0, inf));

	EXPECT_TRUE(Prio(1, 0) < Prio(0, 0));  // higher depth is smaller
	EXPECT_TRUE(Prio(1, 42) < Prio(0, 0));
	EXPECT_TRUE(Prio(1, inf) > Prio(0, 0));  // infinite costs are always largest
	EXPECT_TRUE(Prio(1, inf) < Prio(0, inf));

	EXPECT_TRUE(Prio(0, 0) < Prio(0, 42));  // higher cost is larger
	EXPECT_TRUE(Prio(0, 0) < Prio(0, inf));
	EXPECT_TRUE(Prio(0, 42) > Prio(0, 0));
	EXPECT_TRUE(Prio(0, inf) > Prio(0, 0));
}

using Prio = InterfaceState::Priority;

// Interface that also stores passed states
class StoringInterface : public Interface
{
	std::vector<std::unique_ptr<InterfaceState>> storage_;

public:
	using Interface::Interface;
	void add(InterfaceState&& state) {
		storage_.emplace_back(std::make_unique<InterfaceState>(std::move(state)));
		Interface::add(*storage_.back());
	}
	std::vector<unsigned int> depths() const {
		std::vector<unsigned int> result;
		std::transform(cbegin(), cend(), std::back_inserter(result),
		               [](const InterfaceState* state) { return state->priority().depth(); });
		return result;
	}
};

TEST(Interface, update) {
	auto ps = std::make_shared<planning_scene::PlanningScene>(getModel());
	StoringInterface i;
	i.add(InterfaceState(ps, Prio(1, 0.0)));
	i.add(InterfaceState(ps, Prio(3, 0.0)));
	EXPECT_THAT(i.depths(), ::testing::ElementsAreArray({ 3, 1 }));

	i.updatePriority(*i.rbegin(), Prio(5, 0.0));
	EXPECT_THAT(i.depths(), ::testing::ElementsAreArray({ 5, 3 }));

	i.updatePriority(*i.begin(), Prio(6, std::numeric_limits<double>::infinity()));
	EXPECT_THAT(i.depths(), ::testing::ElementsAreArray({ 5, 3 }));  // larger priority is ignored
}