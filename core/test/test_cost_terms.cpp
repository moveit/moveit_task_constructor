#include "models.h"

#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stages/forward.h>

#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/PoseStamped.h>

#include <gtest/gtest.h>

#include <algorithm>

using namespace moveit::task_constructor;
using namespace planning_scene;

const double STAGE_COST{ 7.0 };

const double TRAJECTORY_DURATION{ 9.0 };

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

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		ps.reset(new PlanningScene(robot_model));
		Generator::init(robot_model);
	}

	bool canCompute() const override { return true; }

	void compute() override {
		InterfaceState state(ps);
		spawn(std::move(state), STAGE_COST);
	}
};

class ConnectMockup : public Connecting
{
	using Connecting::Connecting;

	void compute(const InterfaceState& from, const InterfaceState& to) override {
		auto solution{ std::make_shared<SubTrajectory>() };
		solution->setCost(STAGE_COST);
		connect(from, to, solution);
	}
};

class ForwardMockup : public PropagatingForward
{
	PlanningScenePtr ps;

public:
	using PropagatingForward::PropagatingForward;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		ps.reset(new PlanningScene(robot_model));
		PropagatingForward::init(robot_model);
	}

	void computeForward(const InterfaceState& from) override {
		SubTrajectory solution;
		auto traj{ std::make_shared<robot_trajectory::RobotTrajectory>(ps->getRobotModel(),
			                                                            ps->getRobotModel()->getJointModelGroups()[0]) };
		traj->addSuffixWayPoint(ps->getCurrentState(), 0.0);
		traj->addSuffixWayPoint(ps->getCurrentState(), TRAJECTORY_DURATION);
		solution.setTrajectory(traj);
		solution.setCost(STAGE_COST);
		InterfaceState to(from);

		sendForward(from, std::move(to), std::move(solution));
	};
};

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

template <typename T>
class Standalone : public T
{
	moveit::core::RobotModelConstPtr robot;
	InterfacePtr dummy;
	planning_scene::PlanningSceneConstPtr ps;
	InterfaceStatePtr state_start, state_end;

public:
	Standalone(const moveit::core::RobotModelConstPtr& robot)
	  : T(), robot(robot), dummy(std::make_shared<Interface>()), ps(new planning_scene::PlanningScene(robot)) {}

	// reset and prepare for a compute step
	void prepare() {
		auto impl{ this->pimpl() };
		this->reset();

		state_start.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));
		state_end.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));

		// infer and setup interface from children
		Stage* s{ this };
		ContainerBase* b;

		while ((b = dynamic_cast<ContainerBase*>(s)))
			s = &(*b->pimpl()->children().front());
		InterfaceFlags start_flags{ s->pimpl()->requiredInterface() & START_IF_MASK };

		s = this;
		while ((b = dynamic_cast<ContainerBase*>(s)))
			s = &(*b->pimpl()->children().back());
		InterfaceFlags end_flags{ s->pimpl()->requiredInterface() & END_IF_MASK };

		InterfaceFlags flags{ start_flags | end_flags };

		impl->setPrevEnds(flags & WRITES_PREV_END ? dummy : nullptr);
		impl->setNextStarts(flags & WRITES_NEXT_START ? dummy : nullptr);
		T::init(robot);
		impl->resolveInterface(flags);

		// feed interfaces as required for one computation
		if (flags & READS_START) {
			impl->starts()->add(*state_start);
		}
		if (flags & READS_END) {
			impl->ends()->add(*state_end);
		}
	}

	void runCompute() { this->pimpl()->runCompute(); }

	void computeWithStages(std::initializer_list<StageUniquePtr> stages) {
		this->clear();

		// initializer_list offers only const access (a known shortcoming in C++)
		for (const auto& stage : stages) {
			this->add(const_cast<StageUniquePtr&&>(stage));
		}

		prepare();
		runCompute();
	}

	void computeWithStageCost(std::initializer_list<StageUniquePtr> stages, const CostTerm& cost_term) {
		this->setCostTerm(nullptr);
		for (const auto& stage : stages)
			const_cast<StageUniquePtr&&>(stage)->setCostTerm(cost_term);
		computeWithStages(stages);
	}

	void computeWithContainerCost(std::initializer_list<StageUniquePtr> stages, const CostTerm& cost_term) {
		this->setCostTerm(cost_term);
		computeWithStages(stages);
	}
};

TEST(CostTerm, CostOverwrite) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	const moveit::core::RobotModelConstPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);

	container.computeWithStages({ std::make_unique<GeneratorMockup>() });
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST) << "return cost of stage by default";

	container.computeWithStageCost({ std::make_unique<GeneratorMockup>() }, nullptr);
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST) << "nullptr cost term forwards cost";

	cost::Constant constant_cost{ 1.0 };
	container.computeWithStageCost({ std::make_unique<GeneratorMockup>() }, constant_cost);
	EXPECT_EQ(container.solutions().front()->cost(), constant_cost.cost) << "custom cost overwrites stage cost";
}

TEST(CostTerm, StageTypes) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	moveit::core::RobotModelPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);

	const cost::Constant constant{ 1.0 };

	// already tested above
	// cont.computeWithStageCost(std::make_unique<GeneratorMockup>(), constant);
	// EXPECT_EQ(cont.solutions().front()->cost(), constant.cost);

	container.computeWithStageCost({ std::make_unique<ConnectMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost) << "custom cost works for connect";

	container.computeWithStageCost({ std::make_unique<ForwardMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost) << "custom cost works for forward propagator";

	container.computeWithStageCost({ std::make_unique<BackwardMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant.cost) << "custom cost works for backward propagator";
}

TEST(CostTerm, ForwardUsesCost) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::Forward> container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant_stage{ 84.0 };
	stage->setCostTerm(constant_stage);

	container.computeWithStages({ std::move(stage) });

	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant_stage.cost) << "Forward forwards children's cost";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant_stage.cost) << "Child cost equals Forward cost";
}

TEST(CostTerm, ForwardOverwritesCost) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::Forward> container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant_stage{ 84.0 };
	stage->setCostTerm(constant_stage);

	cost::Constant constant_forward{ 72.0 };
	container.setCostTerm(constant_forward);

	container.computeWithStages({ std::move(stage) });
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant_forward.cost) << "Forward can apply custom cost";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant_stage.cost) << "child's cost is not affected";
}

TEST(CostTerm, ForwardCanModifyCost) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::Forward> container(robot);

	auto stage{ std::make_unique<BackwardMockup>() };
	cost::Constant constant{ 8.0 };
	stage->setCostTerm(constant);
	container.setCostTerm([](auto&& s) { return s.cost() * s.cost(); });

	container.computeWithStages({ std::move(stage) });
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant.cost * constant.cost) << "Forward can compute cost based on child";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant.cost) << "child's cost is not affected";
}

TEST(CostTerm, CompositeSolutions) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	Standalone<SerialContainer> container{ getModel() };

	{
		auto s1{ std::make_unique<ForwardMockup>() };
		auto s2{ std::make_unique<ForwardMockup>() };

		container.computeWithStages({ std::move(s1), std::move(s2) });
		EXPECT_EQ(container.solutions().front()->cost(), 2 * STAGE_COST) << "by default stage costs are added";
	}

	{
		auto s1{ std::make_unique<ForwardMockup>() };
		cost::Constant constant{ 1.0 };
		s1->setCostTerm(constant);
		auto s2{ std::make_unique<ForwardMockup>() };

		container.computeWithStages({ std::move(s1), std::move(s2) });
		EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST + constant.cost)
		    << "mix of explicit and implicit cost terms works";
	}

	{
		auto s1{ std::make_unique<ForwardMockup>() };
		auto s2{ std::make_unique<ForwardMockup>() };
		auto c1{ std::make_unique<SerialContainer>() };
		cost::Constant constant1{ 1.0 };
		s1->setCostTerm(constant1);
		s2->setCostTerm(constant1);
		c1->add(std::move(s1));
		c1->add(std::move(s2));

		auto s3{ std::make_unique<ForwardMockup>() };
		cost::Constant constant2{ 9.0 };
		s3->setCostTerm(constant2);

		container.computeWithStages({ std::move(c1), std::move(s3) });
		EXPECT_EQ(container.solutions().front()->cost(), 2 * constant1.cost + constant2.cost)
		    << "cost aggregation works across multiple levels";
	}
}

TEST(CostTerm, CompositeSolutionsContainerCost) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	Standalone<SerialContainer> container{ getModel() };

	auto s1{ std::make_unique<ForwardMockup>() };
	auto s1_ptr{ s1.get() };
	auto s2{ std::make_unique<ForwardMockup>() };

	auto c1{ std::make_unique<SerialContainer>() };
	c1->add(std::move(s1));
	c1->add(std::move(s2));

	auto s3{ std::make_unique<ForwardMockup>() };

	container.setCostTerm(cost::TrajectoryDuration);
	container.computeWithStages({ std::move(c1), std::move(s3) });
	EXPECT_EQ(container.solutions().front()->cost(), 3 * TRAJECTORY_DURATION)
	    << "container cost term overwrites stage costs";
	EXPECT_EQ(s1_ptr->solutions().front()->cost(), STAGE_COST) << "child cost is not affected";
}

TEST(CostTerm, CompositeSolutionsContainerAggregator) {
	ros::console::set_logger_level(ROSCONSOLE_ROOT_LOGGER_NAME, ros::console::levels::Fatal);
	Standalone<SerialContainer> container{ getModel() };

	cost::Constant constant{ 17.0 };

	{
		auto s1{ std::make_unique<ForwardMockup>() };
		auto s2{ std::make_unique<ForwardMockup>() };
		auto s3{ std::make_unique<ForwardMockup>() };

		container.computeWithStageCost({ std::move(s1), std::move(s2), std::move(s3) }, constant);
		EXPECT_EQ(container.solutions().front()->cost(), 3 * constant.cost) << "default cost aggregates through stages";
	}

	{
		container.setCostAggregator([](double a, double b) { return std::max(a, b); });
		container.computeWithContainerCost(
		    {
		        std::make_unique<ForwardMockup>(),
		        std::make_unique<ForwardMockup>(),
		        std::make_unique<ForwardMockup>(),
		    },
		    constant);

		EXPECT_EQ(container.solutions().front()->cost(), constant.cost)
		    << "custom aggregator works with explicit cost term in container";
	}

	{
		container.setCostAggregator([](auto a, auto b) { return std::max(a, b); });
		container.computeWithStageCost(
		    {
		        std::make_unique<ForwardMockup>(),
		        std::make_unique<ForwardMockup>(),
		        std::make_unique<ForwardMockup>(),
		    },
		    constant);

		EXPECT_EQ(container.solutions().front()->cost(), constant.cost)
		    << "custom aggregator works without container cost term";
	}
}
