#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stage_p.h>

#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

class TestGenerator : public Generator {
public:
	TestGenerator() : Generator("generator") {}
	bool canCompute() const override { return true; }
	bool compute() override { return false; }
};

class TestPropagatingForward : public PropagatingForward {
public:
	TestPropagatingForward() : PropagatingForward("forwarder") {}
	bool computeForward(const InterfaceState &from) override { return false; }
};


class BaseTest : public ::testing::Test {
protected:
	void SetUp() {}
	void TearDown() {}

	void validateOrder(const SerialContainerPrivate* container, const std::initializer_list<StagePrivate*> &expected) {
		size_t num = container->children().size();
		ASSERT_TRUE(num == expected.size()) << "different list lengths";

		// validate position()
		EXPECT_EQ(container->children().begin(), container->position(-(num+1)));
		EXPECT_EQ(container->children().end(), container->position(num));

		// validate order
		size_t pos = 0;
		auto exp_it = expected.begin();
		for (auto it = container->children().begin(), end = container->children().end(); it != end; ++it, ++exp_it, ++pos) {
			StagePrivate *child = (*it)->pimpl();
			EXPECT_EQ(child, *exp_it) << "wrong order";
			EXPECT_EQ(child->parent()->pimpl(), container) << "wrong parent";
			EXPECT_EQ(it, container->position(pos)) << "bad forward position resolution";
			EXPECT_EQ(it, container->position(pos-num-1)) << "bad backward position resolution";
		}
	}
};

#define VALIDATE(...) {\
	SCOPED_TRACE("validateOrder({" #__VA_ARGS__ "})"); \
	validateOrder(cp, {__VA_ARGS__}); \
}

TEST_F(BaseTest, serialContainer) {
	SerialContainer c("serial");
	SerialContainerPrivate *cp = c.pimpl();
	// pretend, that the container is connected
	InterfacePtr dummy(new Interface);
	cp->setNextStarts(dummy);
	cp->setPrevEnds(dummy);
	planning_scene::PlanningScenePtr scene;

	EXPECT_EQ(cp->parent(), nullptr);
	EXPECT_THROW(c.init(scene), InitStageException);
	VALIDATE();

	/*****  inserting first stage  *****/
	auto g = std::make_unique<TestGenerator>();
	GeneratorPrivate *gp = g->pimpl();
	ASSERT_TRUE(c.insert(std::move(g)));
	EXPECT_FALSE(g); // ownership transferred to container
	VALIDATE(gp);

	/*****  inserting second stage  *****/
	auto f = std::make_unique<TestPropagatingForward>();
	PropagatingForwardPrivate *fp = f->pimpl();
	ASSERT_TRUE(c.insert(std::move(f)));
	EXPECT_FALSE(f); // ownership transferred to container
	VALIDATE(gp, fp);

	/*****  inserting third stage  *****/
	auto f2 = std::make_unique<TestPropagatingForward>();
	PropagatingForwardPrivate *fp2 = f2->pimpl();
	ASSERT_TRUE(c.insert(std::move(f2), 1));
	EXPECT_FALSE(f2); // ownership transferred to container
	VALIDATE(gp, fp2, fp);

	EXPECT_NO_THROW(c.init(scene));

	/*****  inserting another generator stage  *****/
	ASSERT_TRUE(c.insert(std::make_unique<TestGenerator>()));
	EXPECT_THROW(c.init(scene), InitStageException);
}
