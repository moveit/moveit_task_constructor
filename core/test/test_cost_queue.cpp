#include <list>
#include <memory>
#include <moveit/task_constructor/cost_queue.h>
#include <moveit/task_constructor/storage.h>
#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

namespace mtc = moveit::task_constructor;

// type-trait functions for OrderedTest<T>
template <typename T>
T create(int cost);
template <>
int create(int cost) {
	return cost;
}
template <>
int* create(int cost) {
	// return new int(cost), but store values for clean memory management
	static std::list<int> storage;
	storage.push_back(cost);
	return &storage.back();
}
template <>
mtc::SolutionBasePtr create(int cost) {
	mtc::SolutionBasePtr s = std::make_shared<mtc::SubTrajectory>();
	s->setCost(cost);
	return s;
}
template <>
mtc::SolutionBaseConstPtr create(int cost) {
	return create<mtc::SolutionBasePtr>(cost);
}

template <typename T>
int value(const T& item);
template <>
int value(const int& item) {
	return item;
}
template <>
int value(int* const& item) {
	return *item;
}
template <>
int value(const mtc::SolutionBasePtr& item) {
	return item->cost();
}
template <>
int value(const mtc::SolutionBaseConstPtr& item) {
	return item->cost();
}

template <typename T>
class ValueOrPointeeLessTest : public ::testing::Test
{
protected:
	bool less(int lhs, int rhs) {
		ValueOrPointeeLess<T> comp;
		return comp(create<T>(lhs), create<T>(rhs));
	}
};
// set of template types to test for
using TypeInstances = ::testing::Types<int, int*, mtc::SolutionBasePtr, mtc::SolutionBaseConstPtr>;
TYPED_TEST_SUITE(ValueOrPointeeLessTest, TypeInstances);
TYPED_TEST(ValueOrPointeeLessTest, less) {
	EXPECT_TRUE(this->less(2, 3));
	EXPECT_FALSE(this->less(1, 1));
	EXPECT_FALSE(this->less(2, 1));
}

template <typename T>
class OrderedTest : public ::testing::Test, public ordered<T>
{
protected:
	void pushAndValidate(int cost, const std::vector<int>& expected) {
		this->insert(create<T>(cost));

		std::vector<int> actual;
		std::transform(this->c.begin(), this->c.end(), std::back_inserter(actual),
		               [](const T& item) -> int { return value<T>(item); });

		EXPECT_THAT(actual, ::testing::ElementsAreArray(expected));
	}
	void validatePop() {
		std::vector<int> expected;
		std::vector<int> actual;
		std::transform(this->c.begin(), this->c.end(), std::back_inserter(expected),
		               [](const T& item) -> int { return value<T>(item); });

		while (!this->empty())
			actual.push_back(value<T>(this->pop()));

		EXPECT_THAT(actual, ::testing::ElementsAreArray(expected));
		EXPECT_TRUE(this->empty());
	}
};

#define pushAndValidate(cost, ...)                                  \
	{                                                                \
		SCOPED_TRACE("pushAndValidate(" #cost ", " #__VA_ARGS__ ")"); \
		this->pushAndValidate(cost, __VA_ARGS__);                     \
	}
TYPED_TEST_SUITE(OrderedTest, TypeInstances);
TYPED_TEST(OrderedTest, sorting) {
	pushAndValidate(2, { 2 });
	pushAndValidate(1, { 1, 2 });
	pushAndValidate(3, { 1, 2, 3 });
	this->validatePop();

	pushAndValidate(1, { 1 });
	pushAndValidate(2, { 1, 2 });
	pushAndValidate(3, { 1, 2, 3 });
	pushAndValidate(4, { 1, 2, 3, 4 });
	pushAndValidate(5, { 1, 2, 3, 4, 5 });
	this->validatePop();

	pushAndValidate(5, { 5 });
	pushAndValidate(4, { 4, 5 });
	pushAndValidate(3, { 3, 4, 5 });
	pushAndValidate(1, { 1, 3, 4, 5 });
	pushAndValidate(2, { 1, 2, 3, 4, 5 });
	this->validatePop();
}

template <typename ValueType, typename CostType>
std::ostream& operator<<(std::ostream& os, const cost_ordered<ValueType, CostType>& queue) {
	for (const auto& pair : queue.sorted())
		os << pair.cost() << ": " << pair.value() << std::endl;
	return os;
}

template <typename ValueType, typename CostType = int>
class CostOrderedTest : public ::testing::Test
{
protected:
	using container_type = std::deque<ValueType>;
	using queue_type = cost_ordered<ValueType, std::deque<ValueType>, CostType>;

	CostOrderedTest() {}

	auto insert(ValueType value, CostType cost = CostType()) { return queue.insert(value, cost); }

	void fill(int first = 1, int last = 5) {
		for (; first < last; ++first)
			insert(first, first);
	}

	cost_ordered<ValueType, CostType> queue;

	void SetUp() override {}
	void TearDown() override {}
};
using CostOrderedTestInt = CostOrderedTest<int, int>;

TEST_F(CostOrderedTestInt, ordered_push) {
	auto top = *insert(2, 2);
	for (int i = 3; i < 6; ++i) {
		insert(i, i);
		EXPECT_EQ(queue.top(), top);
	}
}

TEST_F(CostOrderedTestInt, reverse_ordered_push) {
	for (int i = 6; i > 3; --i) {
		insert(i, i);
		EXPECT_EQ(queue.top().cost(), i);
		EXPECT_EQ(queue.top().value(), i);
	}
}

TEST_F(CostOrderedTestInt, update) {
	fill(3, 5);
	auto first = queue.top();
	auto added = *insert(7, 3);
	// items with same cost are added behind existing values
	EXPECT_EQ(queue.top(), first);
	EXPECT_EQ(*(++queue.begin()), added);
}
