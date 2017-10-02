#include <deque>
#include <moveit_task_constructor/cost_queue.h>
#include <gtest/gtest.h>

template<typename ContainerType, typename CostType, typename Compare>
std::ostream& operator<<(std::ostream &os, const cost_ordered<ContainerType, CostType, Compare> &queue) {
	for (const auto &pair : queue.sorted())
		os << pair.cost() << ": " << pair.value() << std::endl;
	return os;
}

template <typename ValueType, typename CostType = int>
class CostOrderedTest : public ::testing::Test {
protected:
	CostOrderedTest() : ::testing::Test(), queue(items)
	{
	}

	auto push(ValueType value, CostType cost = CostType()) {
		return queue.push_back(items.insert(items.end(), value), cost);
	}

	void fill(int first=1, int last=5) {
		for (; first < last; ++first)
			push(first, first);
	}

	std::deque<ValueType> items;
	cost_ordered<std::deque<ValueType>, CostType> queue;

	void SetUp() {}
	void TearDown() {}
};
typedef CostOrderedTest<int, int> CostOrderedTestInt;

TEST_F(CostOrderedTestInt, ordered_push) {
	auto top = *push(2,2);
	for (int i = 3; i < 6; ++i) {
		push(i, i);
		EXPECT_EQ(queue.top(), top);
	}
}

TEST_F(CostOrderedTestInt, reverse_ordered_push) {
	for (int i = 6; i > 3; --i) {
		push(i, i);
		EXPECT_EQ(queue.top().cost(), i);
		EXPECT_EQ(queue.top().value(), i);
	}
}

TEST_F(CostOrderedTestInt, update) {
	fill(3, 5);
	auto first = queue.sorted().begin();
	auto added = push(7, 3);
	// items with same cost are added behind existing values
	EXPECT_EQ(queue.top(), *first);
	EXPECT_EQ(*(++queue.sorted().begin()), *added);
}
