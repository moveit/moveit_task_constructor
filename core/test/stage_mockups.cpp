#include <moveit/task_constructor/stage_p.h>
#include <moveit/planning_scene/planning_scene.h>

#include "stage_mockups.h"

namespace moveit {
namespace task_constructor {

unsigned int GeneratorMockup::id_ = 0;
unsigned int MonitoringGeneratorMockup::id_ = 0;
unsigned int ConnectMockup::id_ = 0;
unsigned int PropagatorMockup::id_ = 0;
unsigned int ForwardMockup::id_ = 0;
unsigned int BackwardMockup::id_ = 0;

void resetMockupIds() {
	GeneratorMockup::id_ = 0;
	MonitoringGeneratorMockup::id_ = 0;
	ConnectMockup::id_ = 0;
	PropagatorMockup::id_ = 0;
	ForwardMockup::id_ = 0;
	BackwardMockup::id_ = 0;
}

PredefinedCosts::PredefinedCosts(std::list<double>&& costs, bool finite)
  : costs_{ std::move(costs) }, finite_{ finite } {}

bool PredefinedCosts::exhausted() const {
	return costs_.empty();
}

double PredefinedCosts::cost() const {
	// keep the tests going, but this should not happen
	EXPECT_GT(costs_.size(), 0u);
	if (costs_.empty())
		return 0.0;

	double c{ costs_.front() };

	if (finite_ || costs_.size() > 1) {
		costs_.pop_front();
	}

	return c;
}

void DelayingWrapper::compute() {
	if (!delay_.empty()) {  // if empty, we don't delay
		if (delay_.front() == 0)
			delay_.pop_front();  // we can compute() now
		else {
			--delay_.front();  // continue delaying
			return;
		}
	}
	// forward to child, eventually generating multiple solutions at once
	WrapperBase::compute();
}

GeneratorMockup::GeneratorMockup(PredefinedCosts&& costs, std::size_t solutions_per_compute)
  : Generator{ "GEN" + std::to_string(++id_) }
  , costs_{ std::move(costs) }
  , solutions_per_compute_{ solutions_per_compute } {}

void GeneratorMockup::init(const moveit::core::RobotModelConstPtr& robot_model) {
	ps_.reset(new planning_scene::PlanningScene(robot_model));
	Generator::init(robot_model);
}

bool GeneratorMockup::canCompute() const {
	// check if runs are being used and if there are still runs left (costs are then never exhausted) or if costs
	// are being used and they are not exhausted yet
	return !costs_.exhausted();
}

void GeneratorMockup::compute() {
	++runs_;

	for (std::size_t i = 0; canCompute() && i < solutions_per_compute_; ++i)
		spawn(InterfaceState(ps_), costs_.cost());
}

MonitoringGeneratorMockup::MonitoringGeneratorMockup(Stage* monitored, PredefinedCosts&& costs)
  : MonitoringGenerator{ "MON" + std::to_string(++id_), monitored }, costs_{ std::move(costs) } {}

void MonitoringGeneratorMockup::onNewSolution(const SolutionBase& s) {
	++runs_;

	spawn(InterfaceState{ s.end()->scene()->diff() }, SubTrajectory{});
}

ConnectMockup::ConnectMockup(PredefinedCosts&& costs)
  : Connecting{ "CON" + std::to_string(++id_) }, costs_{ std::move(costs) } {}

void ConnectMockup::compute(const InterfaceState& from, const InterfaceState& to) {
	++runs_;

	auto solution{ std::make_shared<SubTrajectory>() };
	solution->setCost(costs_.cost());
	solution->setComment(std::to_string(from.priority().cost()) + " -> " + std::to_string(to.priority().cost()));
	connect(from, to, solution);
}

PropagatorMockup::PropagatorMockup(PredefinedCosts&& costs, std::size_t solutions_per_compute)
  : PropagatingEitherWay{ "PRO" + std::to_string(++id_) }
  , costs_{ std::move(costs) }
  , solutions_per_compute_{ solutions_per_compute } {}

void PropagatorMockup::computeForward(const InterfaceState& from) {
	++runs_;

	for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
		SubTrajectory solution{ robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost() };
		sendForward(from, InterfaceState{ from.scene()->diff() }, std::move(solution));
	}
}

void PropagatorMockup::computeBackward(const InterfaceState& to) {
	++runs_;

	for (std::size_t i = 0; i < solutions_per_compute_; ++i) {
		SubTrajectory solution(robot_trajectory::RobotTrajectoryConstPtr(), costs_.cost());
		sendBackward(InterfaceState(to.scene()->diff()), to, std::move(solution));
	}
}

ForwardMockup::ForwardMockup(PredefinedCosts&& costs, std::size_t solutions_per_compute)
  : PropagatorMockup{ std::move(costs), solutions_per_compute } {
	restrictDirection(FORWARD);
	setName("FWD" + std::to_string(++id_));
}

BackwardMockup::BackwardMockup(PredefinedCosts&& costs, std::size_t solutions_per_compute)
  : PropagatorMockup{ std::move(costs), solutions_per_compute } {
	restrictDirection(BACKWARD);
	setName("BWD" + std::to_string(++id_));
}

}  // namespace task_constructor
}  // namespace moveit
