#pragma once

#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
namespace cost {

/// These structures all implement the Stage::CostTerm API and can be configured via Stage::setCostTerm()
/// add a constant cost to each solution
struct ConstantCost
{
public:
	ConstantCost(double c) : cost(c) {}

	double operator()(const SubTrajectory&, std::string& /* unused */) const { return cost; }

	double cost;
};

/// execution duration of the whole trajectory
double PathLengthCost(const SubTrajectory& s);

/// distance to self-collision
double ClearanceCost(const SubTrajectory& s, std::string& comment);
}
}
}
