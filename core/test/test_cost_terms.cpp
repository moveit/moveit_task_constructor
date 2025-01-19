#include "models.h"

#include <moveit/task_constructor/cost_terms.h>
#include <moveit/task_constructor/stage_p.h>
#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/stages/passthrough.h>

#include <moveit/planning_scene/planning_scene.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <gtest/gtest.h>

#include "stage_mockups.h"

using namespace moveit::task_constructor;
using namespace planning_scene;

constexpr double STAGE_COST{ 7.0 };
constexpr double TERM_COST{ 1.0 };

const double TRAJECTORY_DURATION{ 9.0 };

struct GeneratorCostMockup : public GeneratorMockup
{
	GeneratorCostMockup() : GeneratorMockup{ PredefinedCosts{ { STAGE_COST }, true } } {}
};

struct ConnectCostMockup : public ConnectMockup
{
	ConnectCostMockup() : ConnectMockup{ PredefinedCosts::constant(STAGE_COST) } {}
};

struct ForwardCostMockup : public ForwardMockup
{
	ForwardCostMockup() : ForwardMockup{ PredefinedCosts::constant(STAGE_COST) } {}
};

struct BackwardCostMockup : public BackwardMockup
{
	BackwardCostMockup() : BackwardMockup{ PredefinedCosts::constant(STAGE_COST) } {}
};

struct ForwardTrajectoryMockup : public ForwardMockup
{
	ForwardTrajectoryMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0),
	                        std::size_t solutions_per_compute = 1)
	  : ForwardMockup{ std::move(costs), solutions_per_compute } {}

	void computeForward(const InterfaceState& from) override {
		++runs_;

		for (size_t i = 0; i < solutions_per_compute_; ++i) {
			SubTrajectory solution;
			auto traj{ std::make_shared<robot_trajectory::RobotTrajectory>(from.scene()->getRobotModel(), nullptr) };
			planning_scene::PlanningScenePtr ps{ from.scene()->diff() };
			traj->addSuffixWayPoint(ps->getCurrentState(), 0.0);
			traj->addSuffixWayPoint(ps->getCurrentState(), TRAJECTORY_DURATION);
			solution.setTrajectory(traj);
			solution.setCost(STAGE_COST);
			InterfaceState to(from);

			sendForward(from, std::move(to), std::move(solution));
		}
	}
};

template <typename T>
class Standalone : public T
{
public:
	moveit::core::RobotModelConstPtr robot;
	InterfacePtr dummy;
	planning_scene::PlanningSceneConstPtr ps;
	InterfaceStatePtr state_start, state_end;

	Standalone(const moveit::core::RobotModelConstPtr& robot)
	  : T(), robot(robot), dummy(std::make_shared<Interface>()), ps(new planning_scene::PlanningScene(robot)) {
		resetMockupIds();
	}

	// reset and prepare for a compute step
	void prepare() {
		auto impl{ this->pimpl() };
		this->reset();

		dummy->clear();
		state_start.reset();
		state_end.reset();

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
			state_start.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));
			impl->starts()->add(*state_start);
		}
		if (flags & READS_END) {
			state_end.reset(new InterfaceState(ps, InterfaceState::Priority(1, 0.0)));
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

	void computeWithStageCost(std::initializer_list<StageUniquePtr> stages, const CostTermConstPtr& cost_term) {
		this->setCostTerm(nullptr);
		for (const auto& stage : stages)
			const_cast<StageUniquePtr&&>(stage)->setCostTerm(cost_term);
		computeWithStages(stages);
	}
};

TEST(CostTerm, SolutionConnected) {
	const moveit::core::RobotModelConstPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);
	auto stage{ std::make_unique<ConnectCostMockup>() };

	// custom CostTerm to verify SubTrajectory is hooked up to its states & creator
	class VerifySolutionCostTerm : public TrajectoryCostTerm
	{
		Standalone<SerialContainer>& container_;
		Stage* creator_;

	public:
		VerifySolutionCostTerm(Standalone<SerialContainer>& container, Stage* creator)
		  : container_{ container }, creator_{ creator } {}

		using TrajectoryCostTerm::operator();
		double operator()(const SubTrajectory& s, std::string& /*comment*/) const override {
			EXPECT_EQ(&*container_.state_start,
			          const_cast<const SerialContainerPrivate*>(container_.pimpl())->internalToExternalMap().at(s.start()))
			    << "SubTrajectory is not connected to its expected start InterfaceState";
			EXPECT_EQ(&*container_.state_end,
			          const_cast<const SerialContainerPrivate*>(container_.pimpl())->internalToExternalMap().at(s.end()))
			    << "SubTrajectory is not connected to its expected end InterfaceState";
			EXPECT_EQ(s.creator(), creator_);
			return TERM_COST;
		}
	};

	stage->setCostTerm(std::make_unique<VerifySolutionCostTerm>(container, &*stage));
	container.computeWithStages({ std::move(stage) });
	EXPECT_EQ(container.solutions().front()->cost(), TERM_COST) << "custom CostTerm overwrites stage cost";
}

TEST(CostTerm, SetLambdaCostTerm) {
	const moveit::core::RobotModelConstPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);

	auto stage{ std::make_unique<GeneratorCostMockup>() };
	stage->setCostTerm([](auto&& /*s*/) { return TERM_COST; });
	container.computeWithStages({ std::move(stage) });
	EXPECT_EQ(container.solutions().front()->cost(), TERM_COST) << "can use simple lambda signature";

	stage = std::make_unique<GeneratorCostMockup>();
	stage->setCostTerm([](auto&& /*s*/, auto&& /*comment*/) { return 1.0; });
	container.computeWithStages({ std::move(stage) });
	EXPECT_EQ(container.solutions().front()->cost(), 1.0) << "can use full lambda signature";

	const std::string cost_term_comment{ "I want the user to see this" };
	stage = std::make_unique<GeneratorCostMockup>();
	stage->setCostTerm([&](auto&& /*s*/, auto&& comment) {
		comment = cost_term_comment;
		return 1.0;
	});
	container.computeWithStages({ std::move(stage) });
	auto sol = std::dynamic_pointer_cast<const SolutionSequence>(container.solutions().front());
	EXPECT_EQ(sol->cost(), 1.0);
	EXPECT_EQ(sol->solutions()[0]->comment(), cost_term_comment) << "can write to comment";
}

TEST(CostTerm, CostOverwrite) {
	const moveit::core::RobotModelConstPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);

	container.computeWithStages({ std::make_unique<GeneratorCostMockup>() });
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST) << "return cost of stage by default";

	container.computeWithStageCost({ std::make_unique<GeneratorCostMockup>() }, nullptr);
	EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST) << "nullptr cost term forwards cost";

	auto constant_cost{ std::make_shared<cost::Constant>(1.0) };
	container.computeWithStageCost({ std::make_unique<GeneratorCostMockup>() }, constant_cost);
	EXPECT_EQ(container.solutions().front()->cost(), constant_cost->cost) << "custom cost overwrites stage cost";
}

TEST(CostTerm, StageTypes) {
	moveit::core::RobotModelPtr robot{ getModel() };

	Standalone<SerialContainer> container(robot);

	auto constant{ std::make_shared<cost::Constant>(1.0) };

	// already tested above
	// cont.computeWithStageCost(std::make_unique<GeneratorMockupCost>(), constant);
	// EXPECT_EQ(cont.solutions().front()->cost(), constant.cost);

	container.computeWithStageCost({ std::make_unique<ConnectCostMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant->cost) << "custom cost works for connect";

	container.computeWithStageCost({ std::make_unique<ForwardCostMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant->cost) << "custom cost works for forward propagator";

	container.computeWithStageCost({ std::make_unique<BackwardCostMockup>() }, constant);
	EXPECT_EQ(container.solutions().front()->cost(), constant->cost) << "custom cost works for backward propagator";
}

TEST(CostTerm, PassThroughUsesCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::PassThrough> container(robot);

	auto stage{ std::make_unique<BackwardCostMockup>() };
	auto constant{ std::make_shared<cost::Constant>(84.0) };
	stage->setCostTerm(constant);

	container.computeWithStages({ std::move(stage) });

	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant->cost) << "PassThrough forwards children's cost";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant->cost) << "Child cost equals PassThrough cost";
}

TEST(CostTerm, PassThroughOverwritesCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::PassThrough> container(robot);

	auto stage{ std::make_unique<BackwardCostMockup>() };
	auto constant_inner{ std::make_shared<cost::Constant>(84.0) };
	stage->setCostTerm(constant_inner);

	auto constant_outer{ std::make_shared<cost::Constant>(72.0) };
	container.setCostTerm(constant_outer);

	container.computeWithStages({ std::move(stage) });
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant_outer->cost) << "PassThrough can apply custom cost";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant_inner->cost) << "child's cost is not affected";
}

TEST(CostTerm, PassThroughCanModifyCost) {
	moveit::core::RobotModelPtr robot{ getModel() };
	Standalone<stages::PassThrough> container(robot);

	auto stage{ std::make_unique<BackwardCostMockup>() };
	auto constant{ std::make_shared<cost::Constant>(8.0) };
	stage->setCostTerm(constant);
	container.setCostTerm([](auto&& s) { return s.cost() * s.cost(); });

	container.computeWithStages({ std::move(stage) });
	auto& wrapped{ dynamic_cast<const WrappedSolution&>(*container.solutions().front()) };
	EXPECT_EQ(wrapped.cost(), constant->cost * constant->cost) << "PassThrough can compute cost based on child";
	EXPECT_EQ(wrapped.wrapped()->cost(), constant->cost) << "child's cost is not affected";
}

TEST(CostTerm, CompositeSolutions) {
	Standalone<SerialContainer> container{ getModel() };

	{
		auto s1{ std::make_unique<ForwardCostMockup>() };
		auto s2{ std::make_unique<ForwardCostMockup>() };

		container.computeWithStages({ std::move(s1), std::move(s2) });
		EXPECT_EQ(container.solutions().front()->cost(), 2 * STAGE_COST) << "by default stage costs are added";
	}

	{
		auto s1{ std::make_unique<ForwardCostMockup>() };
		auto constant{ std::make_shared<cost::Constant>(1.0) };
		s1->setCostTerm(constant);
		auto s2{ std::make_unique<ForwardCostMockup>() };

		container.computeWithStages({ std::move(s1), std::move(s2) });
		EXPECT_EQ(container.solutions().front()->cost(), STAGE_COST + constant->cost)
		    << "mix of explicit and implicit cost terms works";
	}

	{
		auto s1{ std::make_unique<ForwardCostMockup>() };
		auto s2{ std::make_unique<ForwardCostMockup>() };
		auto c1{ std::make_unique<SerialContainer>() };
		auto constant1{ std::make_shared<cost::Constant>(1.0) };
		s1->setCostTerm(constant1);
		s2->setCostTerm(constant1);
		c1->add(std::move(s1));
		c1->add(std::move(s2));

		auto s3{ std::make_unique<ForwardCostMockup>() };
		auto constant2{ std::make_shared<cost::Constant>(9.0) };
		s3->setCostTerm(constant2);

		container.computeWithStages({ std::move(c1), std::move(s3) });
		EXPECT_EQ(container.solutions().front()->cost(), 2 * constant1->cost + constant2->cost)
		    << "cost aggregation works across multiple levels";
	}
}

TEST(CostTerm, CompositeSolutionsContainerCost) {
	Standalone<SerialContainer> container{ getModel() };

	auto s1{ std::make_unique<ForwardTrajectoryMockup>() };
	auto s1_ptr{ s1.get() };
	auto s2{ std::make_unique<ForwardTrajectoryMockup>() };

	auto c1{ std::make_unique<SerialContainer>() };
	c1->add(std::move(s1));
	c1->add(std::move(s2));

	auto s3{ std::make_unique<ForwardTrajectoryMockup>() };

	container.setCostTerm(std::make_unique<cost::TrajectoryDuration>());
	container.computeWithStages({ std::move(c1), std::move(s3) });
	EXPECT_EQ(container.solutions().front()->cost(), 3 * TRAJECTORY_DURATION)
	    << "container cost term overwrites stage costs";
	EXPECT_EQ(s1_ptr->solutions().front()->cost(), STAGE_COST) << "child cost is not affected";
}
