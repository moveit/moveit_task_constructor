#pragma once

#include <moveit/task_constructor/storage.h>

namespace moveit {
namespace task_constructor {
/// These structures all implement the Stage::CostTerm API and can be configured via Stage::setCostTerm()
namespace cost {

/// add a constant cost to each solution
struct Constant
{
public:
	Constant(double c) : cost(c) {}

	double operator()(const SubTrajectory&, std::string& /* unused */) const { return cost; }

	double cost;
};

/// trajectory length (interpolated between waypoints)
double PathLength(const SubTrajectory& s);

/// execution duration of the whole trajectory
double TrajectoryDuration(const SubTrajectory& s);

struct LinkMotion
{
	LinkMotion(std::string link_name) : link_name(link_name) {}

	double operator()(const SubTrajectory&, std::string&);

	std::string link_name;
};

/** inverse distance to collision
 *
 * \arg interface compute distances using START or END interface of solution
 * \arg with_world check distances to world objects as well or only look at self-collisions
 * \arg cumulative if true, compute clearance as aggregated distance of all bodies
 * \arg group_property the name of the property which defines the group to look at
 * */
struct Clearance
{
	Clearance(Interface::Direction interface = Interface::START, bool with_world = true, bool cumulative = false,
	          std::string group_property = "group")
	  : interface(interface), with_world(with_world), group_property(group_property) {}

	double operator()(const SubTrajectory& s, std::string& comment);

	Interface::Direction interface;
	bool with_world;
	bool cumulative;
	std::string group_property;
};
}
}
}
