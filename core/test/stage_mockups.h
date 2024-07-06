#pragma once

#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/cost_terms.h>

#include <moveit/planning_scene/planning_scene.h>

#include "models.h"

#include <gtest/gtest.h>
#include <gmock/gmock.h>

namespace moveit {
namespace task_constructor {

MOVEIT_STRUCT_FORWARD(PredefinedCosts);
struct PredefinedCosts : CostTerm
{
	mutable std::list<double> costs_;  // list of costs to assign
	mutable bool finite_;

	PredefinedCosts(std::list<double>&& costs, bool finite = true);

	static PredefinedCosts constant(double c) { return PredefinedCosts{ std::list<double>{ c }, false }; }
	static PredefinedCosts single(double c) { return PredefinedCosts{ std::list<double>{ c }, true }; }

	bool exhausted() const;
	double cost() const;

	double operator()(const SubTrajectory& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const SolutionSequence& /*s*/, std::string& /*comment*/) const override { return cost(); }
	double operator()(const WrappedSolution& /*s*/, std::string& /*comment*/) const override { return cost(); }
};

constexpr double INF{ std::numeric_limits<double>::infinity() };

/* wrapper stage to delay solutions by a given number of steps */
struct DelayingWrapper : public WrapperBase
{
	std::list<unsigned int> delay_;
	/* delay list specifies the number of steps each received solution should be delayed */
	DelayingWrapper(std::list<unsigned int> delay, Stage::pointer&& child)
	  : WrapperBase("delayer", std::move(child)), delay_{ std::move(delay) } {}

	void compute() override;
	void onNewSolution(const SolutionBase& s) override { liftSolution(s); }
};

struct GeneratorMockup : public Generator
{
	planning_scene::PlanningScenePtr ps_;

	PredefinedCosts costs_;
	size_t runs_{ 0 };
	std::size_t solutions_per_compute_;

	static unsigned int id_;

	// default to one solution to avoid infinity loops
	GeneratorMockup(PredefinedCosts&& costs = PredefinedCosts{ std::list<double>{ 0.0 }, true },
	                std::size_t solutions_per_compute = 1);
	GeneratorMockup(std::initializer_list<double> costs, std::size_t solutions_per_compute = 1)
	  : GeneratorMockup{ PredefinedCosts{ std::list<double>{ costs }, true }, solutions_per_compute } {}

	void init(const moveit::core::RobotModelConstPtr& robot_model) override;
	bool canCompute() const override;
	void compute() override;
};

struct MonitoringGeneratorMockup : public MonitoringGenerator
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	MonitoringGeneratorMockup(Stage* monitored, PredefinedCosts&& costs = PredefinedCosts::constant(0.0));
	MonitoringGeneratorMockup(Stage* monitored, std::initializer_list<double> costs)
	  : MonitoringGeneratorMockup{ monitored, PredefinedCosts{ std::list<double>{ costs }, true } } {}

	bool canCompute() const override { return false; }
	void compute() override {}
	void onNewSolution(const SolutionBase& s) override;
};

struct ConnectMockup : public Connecting
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };

	static unsigned int id_;

	ConnectMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0));
	ConnectMockup(std::initializer_list<double> costs)
	  : ConnectMockup{ PredefinedCosts{ std::list<double>{ costs }, true } } {}

	using Connecting::compatible;  // make this accessible for testing

	void compute(const InterfaceState& from, const InterfaceState& to) override;
};

struct PropagatorMockup : public PropagatingEitherWay
{
	PredefinedCosts costs_;
	size_t runs_{ 0 };
	std::size_t solutions_per_compute_;

	static unsigned int id_;

	PropagatorMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	PropagatorMockup(std::initializer_list<double> costs)
	  : PropagatorMockup{ PredefinedCosts{ std::list<double>{ costs }, true } } {}

	void computeForward(const InterfaceState& from) override;
	void computeBackward(const InterfaceState& to) override;
};

struct ForwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	ForwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	ForwardMockup(std::initializer_list<double> costs)
	  : ForwardMockup{ PredefinedCosts{ std::list<double>{ costs }, true } } {}
};

struct BackwardMockup : public PropagatorMockup
{
	static unsigned int id_;

	BackwardMockup(PredefinedCosts&& costs = PredefinedCosts::constant(0.0), std::size_t solutions_per_compute = 1);
	BackwardMockup(std::initializer_list<double> costs)
	  : BackwardMockup{ PredefinedCosts{ std::list<double>{ costs }, true } } {}
};

// reset ids of all Mockup types (used to generate unique stage names)
void resetMockupIds();

// provide a basic test fixture that prepares a Task
struct TaskTestBase : public testing::Test
{
	Task t;
	TaskTestBase() {
		resetMockupIds();
		t.setRobotModel(getModel());
	}

	template <typename C, typename S>
	auto add(C& container, S* stage) -> S* {
		container.add(Stage::pointer(stage));
		return stage;
	}
};

#define EXPECT_COSTS(value, matcher)                                           \
	{                                                                           \
		std ::vector<double> costs;                                              \
		std::transform(value.begin(), value.end(), std::back_inserter(costs),    \
		               [](const SolutionBaseConstPtr& s) { return s->cost(); }); \
		EXPECT_THAT(costs, matcher);                                             \
	}

}  // namespace task_constructor
}  // namespace moveit
