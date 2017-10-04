#include <container_p.h>
#include <subtask_p.h>

#include <gtest/gtest.h>
#include <initializer_list>

namespace testing { namespace internal {
enum GTestColor {
	COLOR_DEFAULT,
	COLOR_RED,
	COLOR_GREEN,
	COLOR_YELLOW
};
extern void ColoredPrintf(GTestColor color, const char* fmt, ...);
} }
#define PRINTF(...)  do { \
	testing::internal::ColoredPrintf(testing::internal::COLOR_YELLOW, __VA_ARGS__); \
} while(0)

namespace moveit { namespace task_constructor {

class TestGenerator : public Generator {
public:
	TestGenerator() : Generator("generator") {}
	bool canCompute() const override { return true; }
	bool compute() override { return false; }
};

class TestPropagatingForward : public PropagatingForward {
public:
	TestPropagatingForward() : PropagatingForward("forwarder") {}
	bool canCompute() const override { return true; }
	bool compute() override { return false; }
};


class BaseTest : public ::testing::Test {
protected:
	void SetUp() {}
	void TearDown() {}

	// accessors for private elements of SubTaskPrivate
	inline const SubTaskPrivate* parent(const SubTaskPrivate* p) const { return p->parent_; }

	void validateOrder(const SerialContainerPrivate* container, const std::initializer_list<SubTaskPrivate*> &expected) {
		size_t num = container->children().size();
		ASSERT_TRUE(num == expected.size()) << "different list lengths";

		// validate position()
		EXPECT_EQ(container->children().begin(), container->position(-(num+1)));
		EXPECT_EQ(container->children().end(), container->position(num));

		// print order
		for (auto it = container->children().begin(), end = container->children().end(); it != end; ++it)
			PRINTF(" %p", (*it)->pimpl_func());
		PRINTF(" *** parent: %p ***\n", container);

		// validate order
		const SubTaskPrivate* predeccessor = container;
		const SubTaskPrivate* successor = container;
		size_t pos = 0;
		auto exp_it = expected.begin();
		for (auto it = container->children().begin(), end = container->children().end(); it != end; ++it, ++exp_it, ++pos) {
			SubTaskPrivate *child = (*it)->pimpl_func();
			EXPECT_EQ(child, *exp_it) << "wrong order";
			EXPECT_EQ(child->parent_, container) << "wrong parent";
			EXPECT_EQ(it, container->position(pos)) << "bad forward position resolution";
			EXPECT_EQ(it, container->position(pos-num-1)) << "bad backward position resolution";
			EXPECT_EQ(container->prev(child), predeccessor) << "wrong link to predeccessor for child " << child;
			if (successor != container)
				EXPECT_EQ(child, successor) << "wrong link to successor for child " << predeccessor;
			// store predeccessor and successor for next round
			predeccessor = child;
			successor = container->next(child);
		}
		EXPECT_EQ(successor, container);
	}
};

TEST_F(BaseTest, interfaceFlags) {
	std::unique_ptr<Generator> g = std::make_unique<TestGenerator>();
	EXPECT_EQ(g->interfaceFlags(), SubTask::InterfaceFlags({SubTask::WRITES_NEXT_INPUT, SubTask::WRITES_PREV_OUTPUT}));
}

#define VALIDATE(...) \
	PRINTF("*** validateOrder({" #__VA_ARGS__ "}) ***"); \
	validateOrder(cp, {__VA_ARGS__});

TEST_F(BaseTest, serialContainer) {
	SerialContainer c("serial");
	SerialContainerPrivate *cp = static_cast<SerialContainerPrivate*>(c.pimpl_func());

	EXPECT_TRUE(bool(cp->input_));
	EXPECT_TRUE(bool(cp->output_));
	EXPECT_EQ(parent(cp), nullptr);
	EXPECT_EQ(parent(cp), nullptr);
	VALIDATE();

	/*****  inserting first stage  *****/
	auto g = std::make_unique<TestGenerator>();
	GeneratorPrivate *gp = static_cast<GeneratorPrivate*>(g->pimpl_func());
	ASSERT_TRUE(c.insert(std::move(g)));
	EXPECT_FALSE(g); // ownership transferred to container
	VALIDATE(gp);

	// inserting another generator should fail
	g = std::make_unique<TestGenerator>();
	EXPECT_FALSE(c.insert(std::move(g)));

	/*****  inserting second stage  *****/
	auto f = std::make_unique<TestPropagatingForward>();
	PropagatingForwardPrivate *fp = static_cast<PropagatingForwardPrivate*>(f->pimpl_func());
	ASSERT_TRUE(c.insert(std::move(f)));
	EXPECT_FALSE(f); // ownership transferred to container
	VALIDATE(gp, fp);

	/*****  inserting third stage  *****/
	auto f2 = std::make_unique<TestPropagatingForward>();
	PropagatingForwardPrivate *fp2 = static_cast<PropagatingForwardPrivate*>(f2->pimpl_func());
	EXPECT_FALSE(c.insert(std::move(f2), 0)); // should fail at first position

	// insert @2nd position
	f2 = std::make_unique<TestPropagatingForward>();
	ASSERT_TRUE(c.insert(std::move(f2), 1));
	EXPECT_FALSE(f2); // ownership transferred to container
	VALIDATE(gp, fp2, fp);
}

} }
