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

/** inverse distance to collision
 *
 * \arg interface compute distances using START or END interface of solution
 * \arg with_world check distances to world objects as well or only look at self-collisions
 * \arg group_property the name of the property which defines the group to look at
 * */
struct ClearanceCost
{
	ClearanceCost(Interface::Direction interface = Interface::START, bool with_world = true,
	              std::string group_property = "group")
	  : interface(interface), with_world(with_world), group_property(group_property) {}

	double operator()(const SubTrajectory& s, std::string& comment);

	Interface::Direction interface;
	bool with_world;
	std::string group_property;
};
}
}
}
