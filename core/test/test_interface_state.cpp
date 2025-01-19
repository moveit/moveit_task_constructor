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
using Prio = InterfaceState::Priority;

TEST(InterfaceStatePriority, compare) {
	EXPECT_TRUE(Prio(0, 0) == Prio(0, 0));
	EXPECT_TRUE(Prio(1, 0) < Prio(0, 0));  // higher depth is smaller
	EXPECT_TRUE(Prio(1, 42) < Prio(0, 0));
	EXPECT_TRUE(Prio(0, 0) < Prio(0, 42));  // at same depth, higher cost is larger

	auto dstart = InterfaceState::Status::ARMED;
	EXPECT_TRUE(Prio(0, 0, dstart) == Prio(0, 0, dstart));
	EXPECT_TRUE(Prio(1, 0, dstart) < Prio(0, 0, dstart));
	EXPECT_TRUE(Prio(1, 42, dstart) < Prio(0, 0, dstart));
	EXPECT_TRUE(Prio(0, 0, dstart) < Prio(0, 42, dstart));

	// disabled prios are always larger than enabled ones
	EXPECT_TRUE(Prio(0, 42) < Prio(1, 0, dstart));
	EXPECT_TRUE(Prio(1, 0) < Prio(0, 42, dstart));

	// other comparison operators
	EXPECT_TRUE(Prio(0, 0) <= Prio(0, 0));
	EXPECT_TRUE(Prio(0, 0) <= Prio(0, 1));
	EXPECT_TRUE(Prio(0, 0) > Prio(1, 10));
	EXPECT_TRUE(Prio(0, 0) >= Prio(0, 0));
	EXPECT_TRUE(Prio(0, 10) >= Prio(0, 0));
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

	i.updatePriority(*i.begin(), Prio(6, 0, InterfaceState::Status::ARMED));
	EXPECT_THAT(i.depths(), ::testing::ElementsAreArray({ 3, 6 }));
}

using PrioPair = std::pair<Prio, Prio>;
inline bool operator<(const PrioPair& lhs, const PrioPair& rhs) {
	return ConnectingPrivate::StatePair::less(lhs.first, lhs.second, rhs.first, rhs.second);
}
PrioPair pair(Prio&& p1, Prio&& p2) {
	return std::make_pair(std::move(p1), std::move(p2));
}
PrioPair pair(InterfaceState::Status s1, InterfaceState::Status s2) {
	return pair(Prio(0, 0, s1), Prio(0, 0, s2));
}
TEST(StatePairs, compare) {
	EXPECT_TRUE(pair(Prio(1, 0), Prio(0, 1)) < pair(Prio(1, 1), Prio(0, 1)));
	EXPECT_TRUE(pair(Prio(1, 1), Prio(1, 1)) < pair(Prio(1, 0), Prio(0, 0)));

	auto good = InterfaceState::Status::ENABLED;
	auto good_good = pair(Prio(0, 10, good), Prio(0, 0, good));
	ASSERT_TRUE(good_good > pair(good, good));  // a bad status should reverse this relation
	for (auto bad : { InterfaceState::Status::ARMED, InterfaceState::Status::PRUNED }) {
		EXPECT_TRUE(good_good < pair(bad, good));
		EXPECT_TRUE(good_good < pair(good, bad));
		EXPECT_TRUE(good_good < pair(bad, bad));
	}
}
