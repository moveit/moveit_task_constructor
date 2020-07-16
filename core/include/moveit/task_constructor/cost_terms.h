/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Hamburg University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Hamburg University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Michael 'v4hn' Goerner
   Desc:   Define implementations for general CostTerm's to use with Stage::setCostTerm()
*/

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
 * \arg with_world check distances to world objects or look at self-collisions
 * \arg cumulative if true, compute clearance as aggregated distance of all bodies
 * \arg group_property the name of the property which defines the group to look at
 * \arg interface compute distances using START or END interface of solution *only*, instead of averaging over
 * trajectory
 * */
struct Clearance
{
	Clearance(bool with_world = true, bool cumulative = false, std::string group_property = "group",
	          Interface::Direction interface = Interface::NONE)
	  : with_world(with_world), cumulative(cumulative), group_property(group_property), interface(interface) {}

	double operator()(const SubTrajectory& s, std::string& comment);

	bool with_world;
	bool cumulative;
	std::string group_property;
	Interface::Direction interface;
};
}
}
}
