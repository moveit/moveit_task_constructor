#include "models.h"

#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stages/forward.h>

#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>

#include <gtest/gtest.h>

using namespace moveit::task_constructor;
using namespace planning_scene;

const double STAGE_COST{ 7.0 };

class GeneratorMockup : public Generator
{
	PlanningScenePtr ps;
	InterfacePtr prev;
	InterfacePtr next;

public:
	GeneratorMockup() : Generator("generator") {
		prev.reset(new Interface);
		next.reset(new Interface);
		pimpl()->setPrevEnds(prev);
		pimpl()->setNextStarts(next);
	}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override { ps.reset(new PlanningScene(robot_model)); }

	bool canCompute() const override { return true; }

	void compute() override {
		InterfaceState state(ps);
		spawn(std::move(state), STAGE_COST);
	}
};
MOVEIT_CLASS_FORWARD(GeneratorMockup);

class ConnectMockup : public Connecting
{
	using Connecting::Connecting;

	void compute(const InterfaceState& from, const InterfaceState& to) override {
		auto solution{ std::make_shared<SubTrajectory>() };
		solution->setCost(STAGE_COST);
		connect(from, to, solution);
	}
};
MOVEIT_CLASS_FORWARD(GeneratorMockup);

class ForwardMockup : public PropagatingForward
{
	using PropagatingForward::PropagatingForward;

	void computeForward(const InterfaceState& from) override {
		SubTrajectory solution;
		solution.setCost(STAGE_COST);
		InterfaceState to(from);

		sendForward(from, std::move(to), std::move(solution));
	};
};
MOVEIT_CLASS_FORWARD(GeneratorMockup);

class BackwardMockup : public PropagatingBackward
{
	using PropagatingBackward::PropagatingBackward;

	void computeBackward(const InterfaceState& to) override {
		SubTrajectory solution;
		solution.setCost(STAGE_COST);
		InterfaceState from(to);

		sendBackward(std::move(from), to, std::move(solution));
	};
};
MOVEIT_CLASS_FORWARD(GeneratorMockup);

class SerialContainerStandalone : public SerialContainer
{
	moveit::core::RobotModelConstPtr robot;
	InterfacePtr dummy;
	planning_scene::PlanningSceneConstPtr ps;
	InterfaceStatePtr state_start, state_end;

public:
	SerialContainerStandalone(const moveit::core::RobotModelConstPtr& robot)
	  : SerialContainer()
	  , robot(robot)
	  , dummy(std::make_shared<Interface>())
	  , ps(new planning_scene::PlanningScene(robot)) {}

	// reset and prepare for a compute step
	void prepare() {
		reset();

		state_start.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));
		state_end.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));

		// infer and setup interface from children
		Stage* s{ this };
		ContainerBase* b;

		while ((b = dynamic_cast<ContainerBase*>(s))) {
			s = &(*b->pimpl()->children().front());
		}
		InterfaceFlags start_flags{ s->pimpl()->requiredInterface() & START_IF_MASK };

		s = this;
		while ((b = dynamic_cast<ContainerBase*>(s))) {
			s = &(*b->pimpl()->children().back());
		}
		InterfaceFlags end_flags{ s->pimpl()->requiredInterface() & END_IF_MASK };

		InterfaceFlags flags{ start_flags | end_flags };

		pimpl()->setPrevEnds(flags & WRITES_PREV_END ? dummy : nullptr);
		pimpl()->setNextStarts(flags & WRITES_NEXT_START ? dummy : nullptr);
		SerialContainer::init(robot);
		pimpl()->resolveInterface(flags);

		// feed interfaces as required for one computation
		if (flags & READS_START) {
			pimpl()->starts()->add(*state_start);
		}
		if (flags & READS_END) {
			pimpl()->ends()->add(*state_end);
		}
	}

	void runCompute() { pimpl()->runCompute(); }

	void computeWithStage(StageUniquePtr&& stage) {
		clear();
		add(std::move(stage));

		prepare();
		runCompute();
	}

	void computeWithStageCost(StageUniquePtr&& stage, const CostTerm& cost_term) {
		stage->setCostTerm(cost_term);
		computeWithStage(std::move(stage));
	}
};

TEST(CostTerm, CostOverwrite) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	const moveit::core::RobotModelConstPtr robot{ getModel() };

	SerialContainerStandalone container(robot);

	// by default return the cost the stage declared
	container.computeWithStage(std::make_unique<GeneratorMockup>());
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST);

	// nullptr is the explicit default
	container.computeWithStageCost(std::make_unique<GeneratorMockup>(), nullptr);
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST);

	// custom cost overwrites the stage cost
	cost::Constant constant_cost{ 1.0 };
	container.computeWithStageCost(std::make_unique<GeneratorMockup>(), constant_cost);
	EXPECT_EQ(container.solutions().front()->cost(), constant_cost.cost);
}

// test all primitive stage types
TEST(CostTerm, StageTypes) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	moveit::core::RobotModelPtr robot{ getModel() };

	SerialContainerStandalone container(robot);

	const cost::Constant constant{ 1.0 };

	// already tested above
	// cont.computeWithStageCost(std::make_unique<GeneratorMockup>(), constant);
	// EXPECT_EQ(cont.solutions().front()->cost(), constant.cost);

	container.computeWithStageCost(std::make_unique<ConnectMockup>(), constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost);

	container.computeWithStageCost(std::make_unique<ForwardMockup>(), constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost);

	container.computeWithStageCost(std::make_unique<BackwardMockup>(), constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost);
}

// Forward uses cost from solution
TEST(CostTerm, ForwardUsesCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	SerialContainerStandalone container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant_stage{ 84.0 };
	stage->setCostTerm(constant_stage);
	auto forward{ std::make_unique<stages::Forward>("forward", std::move(stage)) };
	auto* forward_ptr{ forward.get() };

	container.computeWithStage(std::move(forward));

	ASSERT_NE(nullptr, dynamic_cast<const WrappedSolution*>(&(*forward_ptr->solutions().front())));

	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*forward_ptr->solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant_stage.cost);
	EXPECT_EQ(wrapped.wrapped()->cost(), constant_stage.cost);
}

// Forward can overwrite cost from solution
TEST(CostTerm, ForwardOverwritesCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	SerialContainerStandalone container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant_stage{ 84.0 };
	stage->setCostTerm(constant_stage);
	auto forward{ std::make_unique<stages::Forward>("forward", std::move(stage)) };
	auto* forward_ptr{ forward.get() };

	cost::Constant constant_forward{ 72.0 };
	forward->setCostTerm(constant_forward);

	container.computeWithStage(std::move(forward));
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*forward_ptr->solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant_forward.cost);
	EXPECT_EQ(wrapped.wrapped()->cost(), constant_stage.cost);

	// Forward can modify cost from solution
}

// Forward can modify cost from solution
TEST(CostTerm, ForwardCanModifyCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	SerialContainerStandalone container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant{ 8.0 };
	stage->setCostTerm(constant);
	auto forward{ std::make_unique<stages::Forward>("forward", std::move(stage)) };
	auto* forward_ptr{ forward.get() };
	forward->setCostTerm([](auto&& s) { return s.cost() * s.cost(); });

	container.computeWithStage(std::move(forward));
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*forward_ptr->solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant.cost * constant.cost);
	EXPECT_EQ(wrapped.wrapped()->cost(), constant.cost);
}
