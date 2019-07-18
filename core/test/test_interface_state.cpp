#include <list>
#include <moveit/task_constructor/storage.h>
#include <gtest/gtest.h>

using namespace moveit::task_constructor;
TEST(InterfaceStatePriority, compare) {
	typedef InterfaceState::Priority Prio;
	const double inf = std::numeric_limits<double>::infinity();

	EXPECT_TRUE(Prio(0, 0) == Prio(0, 0));
	EXPECT_TRUE(Prio(0, inf) == Prio(0, inf));

	EXPECT_TRUE(Prio(1, 0) < Prio(0, 0));  // higher depth is smaller
	EXPECT_TRUE(Prio(1, inf) < Prio(0, inf));

	EXPECT_TRUE(Prio(0, 0) < Prio(0, 1));  // higher cost is larger
	EXPECT_TRUE(Prio(0, 0) < Prio(0, inf));
	EXPECT_TRUE(Prio(0, inf) > Prio(0, 0));
}
