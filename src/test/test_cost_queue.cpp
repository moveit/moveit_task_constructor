#include <list>
#include <moveit_task_constructor/cost_queue.h>
#include <gtest/gtest.h>

template<typename ValueType, typename ContainerType, typename CostType, typename Compare>
std::ostream& operator<<(std::ostream &os, const cost_ordered<ValueType, ContainerType, CostType, Compare> &queue) {
	for (const auto &pair : queue.sorted())
		os << pair.cost() << ": " << pair.value() << std::endl;
	return os;
}

template <typename ValueType, typename CostType = int>
class CostOrderedTest : public ::testing::TestWithParam<bool> {
protected:
	typedef std::deque<ValueType> container_type;
	typedef cost_ordered<ValueType, std::deque<ValueType>, CostType> queue_type;

	CostOrderedTest() {
		using_items = GetParam();
		if (using_items) // use externally managed items?
			this->queue = queue_type(this->items);
	}

	auto push(ValueType value, CostType cost = CostType()) {
		return queue.push_back(value, cost);
	}

	void fill(int first=1, int last=5) {
		for (; first < last; ++first)
			push(first, first);
	}

	container_type items; // mimic externally managed items
	cost_ordered<ValueType, std::deque<ValueType>, CostType> queue;
	bool using_items;

	void SetUp() {}
	void TearDown() {}
};
typedef CostOrderedTest<int, int> CostOrderedTestInt;

TEST(CostOrderedConstructor, external) {
	std::deque<int> ext;
	cost_ordered<int> foo(ext);
	EXPECT_EQ(&foo.items(), &ext);

	foo.push_back(1, 3.14);
	EXPECT_EQ(foo.items().size(), 1);
	EXPECT_EQ(ext.size(), 1);
}

TEST(CostOrderedConstructor, internal) {
	cost_ordered<int> foo;
	foo.push_back(1, 3.14);
	EXPECT_EQ(foo.items().size(), 1);
}

TEST_P(CostOrderedTestInt, ordered_push) {
	RecordProperty("container", this->using_items ? "external" : "internal");
	auto top = *push(2,2);
	for (int i = 3; i < 6; ++i) {
		push(i, i);
		EXPECT_EQ(queue.top(), top);
	}
}

TEST_P(CostOrderedTestInt, reverse_ordered_push) {
	for (int i = 6; i > 3; --i) {
		push(i, i);
		EXPECT_EQ(queue.top().cost(), i);
		EXPECT_EQ(queue.top().value(), i);
	}
}

TEST_P(CostOrderedTestInt, update) {
	fill(3, 5);
	auto first = queue.sorted().begin();
	auto added = push(7, 3);
	// items with same cost are added behind existing values
	EXPECT_EQ(queue.top(), *first);
	EXPECT_EQ(*(++queue.sorted().begin()), *added);

	if (using_items) EXPECT_EQ(items.size(), queue.size());
	else EXPECT_EQ(items.size(), 0);
}

INSTANTIATE_TEST_CASE_P(UnManaged,
                        CostOrderedTestInt, ::testing::Values(true, false));

#if 0
TEST(Make, CostOrdered) {
	auto q1 = make_cost_ordered<int>(std::deque<float>());
	auto q2 = make_cost_ordered(std::list<int>());
}
#endif
