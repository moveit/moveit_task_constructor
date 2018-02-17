#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stage_p.h>

#include "gtest_value_printers.h"
#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

class GeneratorMockup : public Generator {
	int runs = 0;
public:
	GeneratorMockup(int runs = 0) : Generator("generator"), runs(runs) {}
	bool canCompute() const override { return runs > 0; }
	bool compute() override { return --runs >= 0 ? true : false; }
};

class PropagatorMockup : public PropagatingEitherWay {
	int fw_runs = 0;
	int bw_runs = 0;
public:
	PropagatorMockup(int fw = 0, int bw = 0) : PropagatingEitherWay("either way"), fw_runs(fw), bw_runs(bw) {}
	bool computeForward(const InterfaceState &from) override { return --fw_runs >= 0 ? true : false; }
	bool computeBackward(const InterfaceState &from) override { return --bw_runs >= 0 ? true : false; }
};
class ForwardMockup : public PropagatorMockup {
public:
	ForwardMockup(int runs = 0) : PropagatorMockup(runs, 0) {
		restrictDirection(FORWARD);
		setName("forward");
	}
};
class BackwardMockup : public PropagatorMockup {
public:
	BackwardMockup(int runs = 0) : PropagatorMockup(0, runs) {
		restrictDirection(BACKWARD);
		setName("backward");
	}
};

class ConnectMockup : public Connecting {
	int runs = 0;
public:
	ConnectMockup(int runs = 0) : Connecting("connect"), runs(runs) {}
	bool compute(const InterfaceState& from, const InterfaceState& to) override { return --runs >= 0 ? true : false; }
};


enum StageType { GEN, FW, BW, ANY, BOTH, CONN };
void append(ContainerBase& c, const std::initializer_list<StageType>& types, int runs = 0) {
	for (StageType t : types) {
		switch (t) {
		case GEN: c.insert(std::make_unique<GeneratorMockup>(runs)); break;
		case FW: c.insert(std::make_unique<ForwardMockup>(runs)); break;
		case BW: c.insert(std::make_unique<BackwardMockup>(runs)); break;
		case ANY: c.insert(std::make_unique<PropagatorMockup>(runs, runs)); break;
		case BOTH: {
			auto s = std::make_unique<PropagatorMockup>(runs, runs);
			s->restrictDirection(PropagatingEitherWay::BOTHWAY);
			c.insert(std::move(s));
			break;
		}
		case CONN: c.insert(std::make_unique<ConnectMockup>(runs)); break;
		}
	}
}


class SerialTest : public ::testing::Test {
protected:
	planning_scene::PlanningScenePtr scene;
	SerialContainer serial;
	InterfacePtr dummy;

	SerialTest() : ::testing::Test(), dummy(new Interface) {}

	void pushInterface(bool start=true, bool end=true) {
		// pretend, that the container is connected
		SerialContainerPrivate *impl = serial.pimpl();
		if (start) impl->setPrevEnds(dummy);
		if (end) impl->setNextStarts(dummy);
	}
	void reset(bool start=true, bool end=true) {
		serial.reset();
		serial.clear();
		SerialContainerPrivate *impl = serial.pimpl();
		impl->setNextStarts(InterfacePtr());
		impl->setPrevEnds(InterfacePtr());
		pushInterface(start, end);
	}

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
	void validateInit(bool start, bool end, const std::initializer_list<StageType>& types, bool expect_failure) {
		reset(start, end);
		append(serial, types);
		try {
			serial.init(scene);
			if (!expect_failure) return; // as expected
			ADD_FAILURE() << "init() didn't recognize a failure condition as expected\n" << serial;
		} catch (const InitStageException &e) {
			if (expect_failure) return; // as expected
			ADD_FAILURE() << "unexpected init failure: \n" << e << "\n" << serial;
		} catch (const std::exception &e) {
			ADD_FAILURE() << "unexpected exception thrown:\n" << e.what();
		} catch (...) {
			ADD_FAILURE() << "unexpected unknown exception thrown";
		}
	}
};

#define VALIDATE(...) {\
	SCOPED_TRACE("validateOrder({" #__VA_ARGS__ "})"); \
	validateOrder(impl, {__VA_ARGS__}); \
}

TEST_F(SerialTest, insertion_order) {
	SerialContainerPrivate *impl = serial.pimpl();

	EXPECT_EQ(impl->parent(), nullptr);
	EXPECT_THROW(serial.init(scene), InitStageException);
	VALIDATE();

	/*****  inserting first stage  *****/
	auto g = std::make_unique<GeneratorMockup>();
	StagePrivate *gp = g->pimpl();
	ASSERT_TRUE(serial.insert(std::move(g)));
	EXPECT_FALSE(g); // ownership transferred to container
	VALIDATE(gp);

	/*****  inserting second stage  *****/
	auto f = std::make_unique<ForwardMockup>();
	StagePrivate *fp = f->pimpl();
	ASSERT_TRUE(serial.insert(std::move(f)));
	EXPECT_FALSE(f); // ownership transferred to container
	VALIDATE(gp, fp);

	/*****  inserting third stage  *****/
	auto f2 = std::make_unique<ForwardMockup>();
	StagePrivate *fp2 = f2->pimpl();
	ASSERT_TRUE(serial.insert(std::move(f2), 1));
	EXPECT_FALSE(f2); // ownership transferred to container
	VALIDATE(gp, fp2, fp);

	/*****  inserting another generator stage  *****/
	auto g2 = std::make_unique<GeneratorMockup>();
	StagePrivate *gp2 = g2->pimpl();
	ASSERT_TRUE(serial.insert(std::move(g2)));
	VALIDATE(gp, fp2, fp, gp2);
}


#define EXPECT_INIT_FAILURE(start, end, ...) {\
	SCOPED_TRACE("validateInit({" #__VA_ARGS__ "})"); \
	validateInit(start, end, {__VA_ARGS__}, true); \
}

#define EXPECT_INIT_SUCCESS(start, end, ...) {\
	SCOPED_TRACE("validateInit({" #__VA_ARGS__ "})"); \
	validateInit(start, end, {__VA_ARGS__}, false); \
}

TEST_F(SerialTest, init_empty) {
	EXPECT_INIT_FAILURE(true, true); // no children
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({WRITES_NEXT_START, WRITES_PREV_END}));

	EXPECT_INIT_FAILURE(false, false); // no children
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags());
}

TEST_F(SerialTest, init_connecting) {
	EXPECT_INIT_SUCCESS(false, false, CONN);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({READS_START, READS_END}));
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_FAILURE(true, true, CONN);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({READS_START, READS_END, WRITES_NEXT_START, WRITES_PREV_END}));

	EXPECT_INIT_FAILURE(false, false, CONN, CONN); // two connecting stages cannot be connected
}

TEST_F(SerialTest, init_generator) {
	EXPECT_INIT_SUCCESS(true, true, GEN);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({WRITES_NEXT_START, WRITES_PREV_END}));
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_FAILURE(false, false, GEN); // generator wants to push, but container cannot propagate pushes

	EXPECT_INIT_FAILURE(true, true, GEN, GEN); // two generator stages cannot be connected
}

TEST_F(SerialTest, init_forward) {
	// container interface doesn't match children
	EXPECT_INIT_FAILURE(false, false, FW);
	EXPECT_INIT_FAILURE(true, true, FW);
	EXPECT_INIT_FAILURE(true, false, FW);

	// these should be fine
	EXPECT_INIT_SUCCESS(false, true, FW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_SUCCESS(false, true, FW, FW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_SUCCESS(true, true, GEN, FW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({WRITES_NEXT_START, WRITES_PREV_END}));
}

TEST_F(SerialTest, init_backward) {
	// container interface doesn't match children
	EXPECT_INIT_FAILURE(false, false, BW);
	EXPECT_INIT_FAILURE(true, true, BW);
	EXPECT_INIT_FAILURE(false, true, BW);

	// these should be fine
	EXPECT_INIT_SUCCESS(true, false, BW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_SUCCESS(true, false, BW, BW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), serial.pimpl()->children().front()->pimpl()->interfaceFlags());

	EXPECT_INIT_SUCCESS(true, true, BW, GEN);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({WRITES_NEXT_START, WRITES_PREV_END}));

	EXPECT_INIT_SUCCESS(true, true, BW, GEN, FW);
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({WRITES_NEXT_START, WRITES_PREV_END}));
}

TEST_F(SerialTest, init_auto) {
	// derive propagation direction from outer interface
	EXPECT_INIT_SUCCESS(false, true, ANY); // -> ->
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({READS_START, WRITES_NEXT_START}));
	EXPECT_INIT_SUCCESS(false, true, ANY, ANY); // <- <-
	EXPECT_EQ(serial.pimpl()->interfaceFlags(), InterfaceFlags({READS_START, WRITES_NEXT_START}));
}
