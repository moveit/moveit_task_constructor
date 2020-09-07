#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/task_p.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/robot_model_test_utils.h>

#include "gtest_value_printers.h"
#include <gtest/gtest.h>
#include <initializer_list>

using namespace moveit::task_constructor;

static unsigned int MOCK_ID = 0;

class GeneratorMockup : public Generator
{
	int runs = 0;

public:
	GeneratorMockup(int runs = 0) : Generator("generator " + std::to_string(++MOCK_ID)), runs(runs) {}
	bool canCompute() const override { return runs > 0; }
	void compute() override {
		if (runs > 0)
			--runs;
	}
};

class MonitoringGeneratorMockup : public MonitoringGenerator
{
public:
	MonitoringGeneratorMockup(Stage* monitored)
	  : MonitoringGenerator("monitoring generator " + std::to_string(++MOCK_ID), monitored) {}
	bool canCompute() const override { return false; }
	void compute() override {}
	void onNewSolution(const SolutionBase& /*solution*/) override {}
};

class PropagatorMockup : public PropagatingEitherWay
{
	int fw_runs = 0;
	int bw_runs = 0;

public:
	PropagatorMockup(int fw = 0, int bw = 0)
	  : PropagatingEitherWay("propagate " + std::to_string(++MOCK_ID)), fw_runs(fw), bw_runs(bw) {}
	void computeForward(const InterfaceState& /* from */) override {
		if (fw_runs > 0)
			--fw_runs;
	}
	void computeBackward(const InterfaceState& /* from */) override {
		if (bw_runs > 0)
			--bw_runs;
	}
};
class ForwardMockup : public PropagatorMockup
{
public:
	ForwardMockup(int runs = 0) : PropagatorMockup(runs, 0) {
		restrictDirection(FORWARD);
		setName("forward " + std::to_string(MOCK_ID));
	}
};
class BackwardMockup : public PropagatorMockup
{
public:
	BackwardMockup(int runs = 0) : PropagatorMockup(0, runs) {
		restrictDirection(BACKWARD);
		setName("backward " + std::to_string(MOCK_ID));
	}
};

class ConnectMockup : public Connecting
{
	int runs = 0;

public:
	ConnectMockup(int runs = 0) : Connecting("connect " + std::to_string(++MOCK_ID)), runs(runs) {}
	void compute(const InterfaceState& /* from */, const InterfaceState& /* to */) override {
		if (runs > 0)
			--runs;
	}
};

enum StageType
{
	GEN,
	FW,
	BW,
	ANY,
	CONN
};
void append(ContainerBase& c, const std::initializer_list<StageType>& types, int runs = 0) {
	for (StageType t : types) {
		switch (t) {
			case GEN:
				c.add(std::make_unique<GeneratorMockup>(runs));
				break;
			case FW:
				c.add(std::make_unique<ForwardMockup>(runs));
				break;
			case BW:
				c.add(std::make_unique<BackwardMockup>(runs));
				break;
			case ANY:
				c.add(std::make_unique<PropagatorMockup>(runs, runs));
				break;
			case CONN:
				c.add(std::make_unique<ConnectMockup>(runs));
				break;
		}
	}
}

class NamedStage : public GeneratorMockup
{
public:
	NamedStage(const std::string& name) : GeneratorMockup() { setName(name); }
};

TEST(ContainerBase, positionForInsert) {
	SerialContainer s;
	SerialContainerPrivate* impl = s.pimpl();

	EXPECT_EQ(impl->childByIndex(0, true), impl->children().end());
	EXPECT_EQ(impl->childByIndex(1, true), impl->children().end());
	EXPECT_EQ(impl->childByIndex(-1, true), impl->children().end());
	EXPECT_EQ(impl->childByIndex(-2, true), impl->children().end());

	s.add(std::make_unique<NamedStage>("0"));
	EXPECT_STREQ((*impl->childByIndex(0, true))->name().c_str(), "0");
	EXPECT_EQ(impl->childByIndex(-1, true), impl->children().end());
	EXPECT_STREQ((*impl->childByIndex(-2, true))->name().c_str(), "0");
	EXPECT_EQ(impl->childByIndex(-3, true), impl->children().end());

	s.add(std::make_unique<NamedStage>("1"));
	EXPECT_STREQ((*impl->childByIndex(0, true))->name().c_str(), "0");
	EXPECT_STREQ((*impl->childByIndex(1, true))->name().c_str(), "1");
	EXPECT_EQ(impl->childByIndex(2, true), impl->children().end());

	EXPECT_EQ(impl->childByIndex(-1, true), impl->children().end());
	EXPECT_STREQ((*impl->childByIndex(-2, true))->name().c_str(), "1");
	EXPECT_STREQ((*impl->childByIndex(-3, true))->name().c_str(), "0");
	EXPECT_EQ(impl->childByIndex(-4, true), impl->children().end());
}

/* TODO: remove interface as it returns raw pointers */
TEST(ContainerBase, findChild) {
	SerialContainer s, *c2;
	Stage *a, *b, *c1, *d;
	s.add(Stage::pointer(a = new NamedStage("a")));
	s.add(Stage::pointer(b = new NamedStage("b")));
	s.add(Stage::pointer(c1 = new NamedStage("c")));
	auto sub = ContainerBase::pointer(c2 = new SerialContainer("c"));
	sub->add(Stage::pointer(d = new NamedStage("d")));
	s.add(std::move(sub));

	EXPECT_EQ(s.findChild("a"), a);
	EXPECT_EQ(s.findChild("b"), b);
	EXPECT_EQ(s.findChild("c"), c1);
	EXPECT_EQ(s.findChild("d"), nullptr);
	EXPECT_EQ(s.findChild("c/d"), d);
}

template <typename Container>
class InitTest : public ::testing::Test
{
protected:
	moveit::core::RobotModelConstPtr robot_model;
	Container container;
	InterfacePtr dummy;

	InitTest() : ::testing::Test(), dummy(new Interface) {}

	void pushInterface(bool start = true, bool end = true) {
		// pretend, that the container is connected
		ContainerBasePrivate* impl = container.pimpl();
		if (start)
			impl->setPrevEnds(dummy);
		if (end)
			impl->setNextStarts(dummy);
	}
	void reset(bool start = true, bool end = true) {
		container.reset();
		ContainerBasePrivate* impl = container.pimpl();
		impl->setNextStarts(InterfacePtr());
		impl->setPrevEnds(InterfacePtr());
		pushInterface(start, end);
	}
	void initContainer(const std::initializer_list<StageType>& types = {}) {
		container.clear();
		MOCK_ID = 0;
		append(container, types);
	}

	void validateInit(bool start, bool end, bool expect_failure) { validateInit(start, end, {}, expect_failure); }

	void validateInit(bool start, bool end, const std::initializer_list<StageType>& types, bool expect_failure) {
		reset(start, end);
		append(container, types);
		try {
			container.init(robot_model);
			// resolve interfaces based on provided external interface (start, end)
			InterfaceFlags expected;
			if (start)
				expected |= WRITES_PREV_END;
			else
				expected |= READS_START;

			if (end)
				expected |= WRITES_NEXT_START;
			else
				expected |= READS_END;

			container.pimpl()->resolveInterface(expected);
			container.pimpl()->validateConnectivity();
			if (!expect_failure)
				return;  // as expected
			ADD_FAILURE() << "init() didn't recognize a failure condition as expected\n" << container;
		} catch (const InitStageException& e) {
			if (expect_failure) {
				std::cout << "Expected " << e << "\n" << container;
				return;  // as expected
			}
			ADD_FAILURE() << "Unexpected " << e << "\n" << container;
		} catch (const std::exception& e) {
			ADD_FAILURE() << "unexpected exception thrown:\n" << e.what();
		} catch (...) {
			ADD_FAILURE() << "unexpected unknown exception thrown";
		}
	}
};

class SerialTest : public InitTest<SerialContainer>
{
protected:
	void validateOrder(const SerialContainerPrivate* container, const std::initializer_list<StagePrivate*>& expected) {
		size_t num = container->children().size();
		ASSERT_TRUE(num == expected.size()) << "different list lengths";

		// validate position()
		EXPECT_EQ(container->children().begin(), container->childByIndex(-num));
		EXPECT_EQ(container->children().end(), container->childByIndex(num));

		// validate order
		size_t pos = 0;
		auto exp_it = expected.begin();
		for (auto it = container->children().begin(), end = container->children().end(); it != end;
		     ++it, ++exp_it, ++pos) {
			StagePrivate* child = (*it)->pimpl();
			EXPECT_EQ(child, *exp_it) << "wrong order";
			EXPECT_EQ(child->parent()->pimpl(), container) << "wrong parent";
			EXPECT_EQ(it, container->childByIndex(pos)) << "bad forward position resolution";
			EXPECT_EQ(it, container->childByIndex(pos - num)) << "bad backward position resolution";
		}
	}
};

#define VALIDATE(...)                                    \
	{                                                     \
		SCOPED_TRACE("validateOrder({" #__VA_ARGS__ "})"); \
		validateOrder(impl, { __VA_ARGS__ });              \
	}

TEST_F(SerialTest, insertion_order) {
	SerialContainerPrivate* impl = container.pimpl();

	EXPECT_EQ(impl->parent(), nullptr);
	EXPECT_THROW(container.init(robot_model), InitStageException);
	VALIDATE();

	/*****  inserting first stage  *****/
	auto g = std::make_unique<GeneratorMockup>();
	StagePrivate* gp = g->pimpl();
	ASSERT_TRUE(container.insert(std::move(g)));
	EXPECT_FALSE(g);  // ownership transferred to container
	VALIDATE(gp);

	/*****  inserting second stage  *****/
	auto f = std::make_unique<ForwardMockup>();
	StagePrivate* fp = f->pimpl();
	ASSERT_TRUE(container.insert(std::move(f)));
	EXPECT_FALSE(f);  // ownership transferred to container
	VALIDATE(gp, fp);

	/*****  inserting third stage  *****/
	auto f2 = std::make_unique<ForwardMockup>();
	StagePrivate* fp2 = f2->pimpl();
	ASSERT_TRUE(container.insert(std::move(f2), 1));
	EXPECT_FALSE(f2);  // ownership transferred to container
	VALIDATE(gp, fp2, fp);

	/*****  inserting another generator stage  *****/
	auto g2 = std::make_unique<GeneratorMockup>();
	StagePrivate* gp2 = g2->pimpl();
	ASSERT_TRUE(container.insert(std::move(g2)));
	VALIDATE(gp, fp2, fp, gp2);
}

#define EXPECT_INIT_FAILURE(start, end, ...)            \
	{                                                    \
		SCOPED_TRACE("validateInit({" #__VA_ARGS__ "})"); \
		initContainer();                                  \
		validateInit(start, end, { __VA_ARGS__ }, true);  \
	}

#define EXPECT_INIT_SUCCESS(start, end, ...)            \
	{                                                    \
		SCOPED_TRACE("validateInit({" #__VA_ARGS__ "})"); \
		initContainer();                                  \
		validateInit(start, end, { __VA_ARGS__ }, false); \
	}

TEST_F(SerialTest, init_empty) {
	EXPECT_INIT_FAILURE(true, true);  // no children
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	EXPECT_INIT_FAILURE(false, false);  // no children
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags());
}

TEST_F(SerialTest, init_connecting) {
	EXPECT_INIT_SUCCESS(false, false, CONN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(CONNECT));

	// external interface does not match CONNECT
	EXPECT_INIT_FAILURE(false, true, CONN);
	EXPECT_INIT_FAILURE(true, false, CONN);
	EXPECT_INIT_FAILURE(true, true, CONN);

	EXPECT_INIT_FAILURE(false, false, CONN, CONN);  // two connecting stages cannot be connected
}

TEST_F(SerialTest, init_generator) {
	EXPECT_INIT_SUCCESS(true, true, GEN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	// external interface does not match GENERATE
	EXPECT_INIT_FAILURE(false, false, GEN);
	EXPECT_INIT_FAILURE(false, true, GEN);
	EXPECT_INIT_FAILURE(true, false, GEN);

	EXPECT_INIT_FAILURE(true, true, GEN, GEN);  // two generator stages cannot be connected
}

TEST_F(SerialTest, keep_configured_propagator_interface) {
	// configuring an ANY propagator should work
	EXPECT_INIT_SUCCESS(false, true, ANY);
	EXPECT_EQ(container.pimpl()->children().front()->pimpl()->requiredInterface(), InterfaceFlags(PROPAGATE_FORWARDS));

	// multiple times
	validateInit(false, true, false);
	EXPECT_EQ(container.pimpl()->children().front()->pimpl()->requiredInterface(), InterfaceFlags(PROPAGATE_FORWARDS));
	// and with another interface direction as well
	validateInit(true, false, false);
	EXPECT_EQ(container.pimpl()->children().front()->pimpl()->requiredInterface(), InterfaceFlags(PROPAGATE_BACKWARDS));

	// configuring a FORWARD propagator should work in forward direction
	EXPECT_INIT_SUCCESS(false, true, FW);
	// but fail when initialized in backward direction
	validateInit(true, false, true);
	EXPECT_EQ(container.pimpl()->children().front()->pimpl()->requiredInterface(), InterfaceFlags(PROPAGATE_FORWARDS));

	// same with BACKWARD propagator
	EXPECT_INIT_SUCCESS(true, false, BW);
	validateInit(false, true, true);
	EXPECT_EQ(container.pimpl()->children().front()->pimpl()->requiredInterface(), InterfaceFlags(PROPAGATE_BACKWARDS));
}

TEST_F(SerialTest, init_forward) {
	// container interface doesn't match children
	EXPECT_INIT_FAILURE(false, false, FW);
	EXPECT_INIT_FAILURE(true, true, FW);
	EXPECT_INIT_FAILURE(true, false, FW);

	// wrong inner connectivity
	EXPECT_INIT_FAILURE(false, false, FW, BW);
	EXPECT_INIT_FAILURE(true, true, BW, FW);

	// these should be fine
	EXPECT_INIT_SUCCESS(false, true, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));

	EXPECT_INIT_SUCCESS(false, true, FW, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));

	EXPECT_INIT_SUCCESS(true, true, GEN, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	EXPECT_INIT_SUCCESS(true, true, BW, GEN, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	EXPECT_INIT_SUCCESS(false, true, ANY, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));

	EXPECT_INIT_SUCCESS(false, true, FW, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));

	EXPECT_INIT_SUCCESS(true, false, ANY, BW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_SUCCESS(true, false, BW, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
}

TEST_F(SerialTest, init_backward) {
	// container interface doesn't match children
	EXPECT_INIT_FAILURE(false, false, BW);
	EXPECT_INIT_FAILURE(true, true, BW);
	EXPECT_INIT_FAILURE(false, true, BW);

	// these should be fine
	EXPECT_INIT_SUCCESS(true, false, BW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_SUCCESS(true, false, BW, BW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_SUCCESS(true, true, BW, GEN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));
}

TEST_F(SerialTest, interface_detection) {
	// ANY resolves in either direction
	EXPECT_INIT_SUCCESS(false, true, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_INIT_SUCCESS(true, false, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_SUCCESS(true, true, GEN, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	EXPECT_INIT_SUCCESS(true, true, ANY, GEN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	// derive propagation direction from inner generator
	EXPECT_INIT_SUCCESS(true, true, ANY, GEN, ANY);  // <- <-> ->
	auto it = container.pimpl()->children().begin();
	EXPECT_EQ((*it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(GENERATE));

	// derive propagation direction from inner connector
	EXPECT_INIT_SUCCESS(false, false, ANY, CONN, ANY);  // -> -- <-
	it = container.pimpl()->children().begin();
	EXPECT_EQ((*it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(CONNECT));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(CONNECT));

	EXPECT_INIT_SUCCESS(false, false, CONN, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(CONNECT));

	EXPECT_INIT_SUCCESS(false, false, ANY, CONN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(CONNECT));

	// derive propagation direction from outer interface
	EXPECT_INIT_SUCCESS(false, true, ANY);  // should be resolved to FW
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_INIT_SUCCESS(true, false, ANY);  // should be resolved to BW
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_SUCCESS(false, true, ANY, ANY);  // -> ->
	it = container.pimpl()->children().begin();
	EXPECT_EQ((*it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));

	EXPECT_INIT_SUCCESS(true, false, ANY, ANY);  // <- <-
	it = container.pimpl()->children().begin();
	EXPECT_EQ((*it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
	EXPECT_EQ((*++it)->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));

	EXPECT_INIT_FAILURE(true, true, ANY, ANY);  // no generator

	EXPECT_INIT_FAILURE(false, false, ANY, ANY);  // no connect
}

template <typename T>
Stage::pointer make(const std::string& name, const std::initializer_list<StageType>& children, int runs = 0) {
	auto container = new T(name);
	append(*container, children);
	return Stage::pointer(container);
}

TEST_F(SerialTest, nested_interface_detection) {
	// direction imposed from outer generator
	initContainer({ GEN, ANY });
	container.add(make<SerialContainer>("inner serial", { ANY, ANY }));
	append(container, { ANY });
	{
		SCOPED_TRACE("GEN - ANY - Serial( ANY - ANY ) - ANY");
		validateInit(true, true, false);
	}

	// direction imposed from inner generator
	initContainer({ ANY });
	container.add(make<SerialContainer>("inner serial", { ANY, GEN, ANY }));
	append(container, { ANY });
	{
		SCOPED_TRACE("ANY - Serial( ANY - GEN - ANY ) - ANY");
		validateInit(true, true, false);
	}

	initContainer();
	container.add(make<SerialContainer>("inner serial", { GEN, ANY }));
	append(container, { ANY });
	{
		SCOPED_TRACE("Serial( GEN - ANY ) - ANY");
		validateInit(true, true, false);
	}

	// outer and inner generators conflict with each other
	initContainer({ GEN, ANY });
	container.add(make<SerialContainer>("inner serial", { ANY, GEN, ANY }));
	append(container, { ANY });
	{
		SCOPED_TRACE("GEN - ANY - Serial( ANY - GEN - ANY ) - ANY");
		validateInit(true, true, true);  // expected to fail
	}
}

TEST_F(SerialTest, nested_parallel) {
	initContainer({ GEN });
	container.add(make<Alternatives>("inner alternatives", { ANY, ANY }));
	{
		SCOPED_TRACE("GEN - Alternatives( ANY - ANY )");
		validateInit(true, true, false);
	}

	initContainer({ GEN });
	container.add(make<Fallbacks>("inner alternatives", { ANY, FW }));
	{
		SCOPED_TRACE("GEN - Fallback( ANY - FW )");
		validateInit(true, true, false);
	}
}

class ParallelTest : public InitTest<Alternatives>
{};

TEST_F(ParallelTest, init_matching) {
	EXPECT_INIT_SUCCESS(true, true, GEN, GEN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), GENERATE);

	EXPECT_INIT_SUCCESS(false, false, CONN, CONN);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), CONNECT);

	// external interface doesn't match derived
	EXPECT_INIT_FAILURE(false, false, GEN, GEN);
	EXPECT_INIT_FAILURE(true, false, GEN, GEN);
	EXPECT_INIT_FAILURE(false, true, GEN, GEN);

	EXPECT_INIT_FAILURE(true, false, CONN, CONN);
	EXPECT_INIT_FAILURE(false, true, CONN, CONN);
	EXPECT_INIT_FAILURE(true, true, CONN, CONN);
}

TEST_F(ParallelTest, init_mismatching) {
	EXPECT_INIT_FAILURE(false, false, BW, CONN);
	EXPECT_INIT_FAILURE(true, false, BW, CONN);
	EXPECT_INIT_FAILURE(false, true, BW, CONN);
	EXPECT_INIT_FAILURE(true, true, BW, CONN);

	EXPECT_INIT_FAILURE(false, false, FW, CONN);
	EXPECT_INIT_FAILURE(true, false, FW, CONN);
	EXPECT_INIT_FAILURE(false, true, FW, CONN);
	EXPECT_INIT_FAILURE(true, true, FW, CONN);

	EXPECT_INIT_FAILURE(false, false, ANY, CONN);
	EXPECT_INIT_FAILURE(true, false, ANY, CONN);
	EXPECT_INIT_FAILURE(false, true, ANY, CONN);
	EXPECT_INIT_FAILURE(true, true, ANY, CONN);

	EXPECT_INIT_FAILURE(false, false, BW, GEN);
	EXPECT_INIT_FAILURE(true, false, BW, GEN);
	EXPECT_INIT_FAILURE(false, true, BW, GEN);
	EXPECT_INIT_FAILURE(true, true, BW, GEN);

	EXPECT_INIT_FAILURE(false, false, FW, GEN);
	EXPECT_INIT_FAILURE(true, false, FW, GEN);
	EXPECT_INIT_FAILURE(false, true, FW, GEN);
	EXPECT_INIT_FAILURE(true, true, FW, GEN);

	EXPECT_INIT_FAILURE(false, false, ANY, GEN);
	EXPECT_INIT_FAILURE(true, false, ANY, GEN);
	EXPECT_INIT_FAILURE(false, true, ANY, GEN);
	EXPECT_INIT_FAILURE(true, true, ANY, GEN);

	EXPECT_INIT_FAILURE(false, false, CONN, GEN);
	EXPECT_INIT_FAILURE(true, false, CONN, GEN);
	EXPECT_INIT_FAILURE(false, true, CONN, GEN);
	EXPECT_INIT_FAILURE(true, true, CONN, GEN);

	EXPECT_INIT_FAILURE(false, false, BW, FW);
	EXPECT_INIT_FAILURE(true, false, BW, FW);
	EXPECT_INIT_FAILURE(false, true, BW, FW);
	EXPECT_INIT_FAILURE(true, true, BW, FW);
}

TEST_F(ParallelTest, init_propagating) {
	EXPECT_INIT_SUCCESS(false, true, FW, FW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), PROPAGATE_FORWARDS);
	EXPECT_INIT_FAILURE(false, false, FW, FW);
	EXPECT_INIT_FAILURE(true, false, FW, FW);
	EXPECT_INIT_FAILURE(true, true, FW, FW);

	EXPECT_INIT_SUCCESS(true, false, BW, BW);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), PROPAGATE_BACKWARDS);
	EXPECT_INIT_FAILURE(false, false, BW, BW);
	EXPECT_INIT_FAILURE(false, true, BW, BW);
	EXPECT_INIT_FAILURE(true, true, BW, BW);
}

TEST_F(ParallelTest, init_any) {
	EXPECT_INIT_FAILURE(true, true, ANY, ANY);  // no generator
	EXPECT_INIT_FAILURE(false, false, ANY, ANY);  // no connector

	// infer container (and children) direction from outside
	EXPECT_INIT_SUCCESS(false, true, ANY, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS));
	for (const auto& child : container.pimpl()->children())
		EXPECT_EQ(child->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS)) << child->name();

	EXPECT_INIT_SUCCESS(true, false, ANY, ANY);
	EXPECT_EQ(container.pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS));
	for (const auto& child : container.pimpl()->children())
		EXPECT_EQ(child->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS)) << child->name();

	EXPECT_INIT_SUCCESS(true, false, BW, ANY);  // infer ANY as BACKWARDS
	for (const auto& child : container.pimpl()->children())
		EXPECT_EQ(child->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_BACKWARDS)) << child->name();

	EXPECT_INIT_SUCCESS(false, true, ANY, FW);  // infer ANY as FORWARDS
	for (const auto& child : container.pimpl()->children())
		EXPECT_EQ(child->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS)) << child->name();

	EXPECT_INIT_SUCCESS(false, true, FW, ANY, FW);  // infer ANY as FORWARDS
	for (const auto& child : container.pimpl()->children())
		EXPECT_EQ(child->pimpl()->interfaceFlags(), InterfaceFlags(PROPAGATE_FORWARDS)) << child->name();
}

TEST(Task, move) {
	Task t1("foo");
	t1.add(std::make_unique<GeneratorMockup>());
	t1.add(std::make_unique<GeneratorMockup>());
	EXPECT_EQ(t1.stages()->numChildren(), 2u);

	Task t2 = std::move(t1);
	EXPECT_EQ(t2.stages()->numChildren(), 2u);
	EXPECT_EQ(t1.stages()->numChildren(), 0u);

	t1 = std::move(t2);
	EXPECT_EQ(t1.stages()->numChildren(), 2u);
	EXPECT_EQ(t2.stages()->numChildren(), 0u);
}

TEST(Task, reuse) {
	// create dummy robot model
	moveit::core::RobotModelBuilder builder("robot", "base");
	builder.addChain("base->a->b->c", "continuous");
	builder.addGroupChain("base", "c", "group");
	moveit::core::RobotModelConstPtr robot_model = builder.build();

	Task t("first");
	t.setRobotModel(robot_model);

	auto configure = [](Task& t) {
		auto ref = new stages::FixedState("fixed");
		auto scene = std::make_shared<planning_scene::PlanningScene>(t.getRobotModel());
		ref->setState(scene);

		t.add(Stage::pointer(ref));
		t.add(std::make_unique<ConnectMockup>());
		t.add(std::make_unique<MonitoringGeneratorMockup>(ref));
	};

	try {
		configure(t);
		t.plan(1);

		t = Task("second");
		t.setRobotModel(robot_model);
		EXPECT_EQ(static_cast<void*>(t.pimpl()->me()), static_cast<void*>(&t));
		EXPECT_EQ(t.pimpl()->children().size(), 1u);
		EXPECT_EQ(static_cast<void*>(t.stages()->pimpl()->parent()), static_cast<void*>(&t));

		configure(t);
		t.plan(1);
	} catch (const InitStageException& e) {
		ADD_FAILURE() << "InitStageException:" << std::endl << e << t;
	}
}
