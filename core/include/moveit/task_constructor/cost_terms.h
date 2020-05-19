#pragma once

#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
namespace cost {

/// These structures all implement the Stage::CostTerm API and can be configured via Stage::setCostTerm()

/// add a constant cost to each solution
class ConstantCost
{
public:
	ConstantCost(double cost) : cost_(cost) {}

	double operator()(const SolutionBase&) { return cost_; }

private:
	double cost_;
};

/// define cost as execution duration of the whole trajectory
double PathLengthCost(const SolutionBase& s);
}
}
}
