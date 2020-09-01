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
#include <moveit/task_constructor/utils.h>

namespace moveit {
namespace task_constructor {

class CostTerm
{
public:
	using SubTrajectorySig = std::function<double(const SubTrajectory&, std::string&)>;
	using SubTrajectoryShortSig = std::function<double(const SubTrajectory&)>;

	// accept lambdas according to either signature above
	template <typename Term>
	CostTerm(const Term& t) : CostTerm{ decltype(sigMatcher(t)){ t } } {}

	CostTerm(const SubTrajectorySig&);
	CostTerm(const SubTrajectoryShortSig&);
	CostTerm(std::nullptr_t);
	CostTerm();

	// is this a valid CostTerm?
	operator bool() const;

	double operator()(const SolutionBase& s, std::string& comment) const;

	/** describes the supported types of solutions that should be forwarded to this CostTerm
	 *
	 * CostTerms that support more than `SubTrajectory`s should inherit and overwrite the internal supports_ flag
	 */
	enum class SolutionType
	{
		NONE = 0,
		TRAJECTORY = 1 << 0,
		SEQUENCE = 1 << 1,
		WRAPPER = 1 << 2,
		ALL = TRAJECTORY | SEQUENCE | WRAPPER
	};
	Flags<SolutionType> supports() const;

protected:
	CostTerm(const std::function<double(const SolutionBase&, std::string&)>&, Flags<SolutionType>);

	std::function<double(const SolutionBase&, std::string&)> term_;
	Flags<SolutionType> supports_;

private:
	template <typename T>
	auto sigMatcher(const T& t) -> decltype(t(SubTrajectory{}), SubTrajectoryShortSig{});
	template <typename T>
	auto sigMatcher(const T& t) -> decltype(t(SubTrajectory{}, std::string{}), SubTrajectorySig{});
};

namespace cost {

/// add a constant cost to each solution
class Constant : public CostTerm
{
public:
	Constant(double c);

	double cost;
};

/// trajectory length (interpolated between waypoints)
class PathLength : public CostTerm
{
public:
	PathLength();
	// TODO(v4hn): allow to consider specific joints only
};

/// execution duration of the whole trajectory
class TrajectoryDuration : public CostTerm
{
public:
	TrajectoryDuration();
};

class LinkMotion : public CostTerm
{
public:
	LinkMotion(std::string link_name);

	std::string link_name;

protected:
	double compute(const SubTrajectory&, std::string&) const;
};

/** inverse distance to collision
 *
 * \arg with_world check distances to world objects or look at self-collisions
 * \arg cumulative if true, compute clearance as aggregated distance of all bodies
 * \arg group_property the name of the property which defines the group to look at
 * \arg interface compute distances using START or END interface of solution *only*, instead of averaging over
 * trajectory
 * */
class Clearance : public CostTerm
{
public:
	Clearance(bool with_world = true, bool cumulative = false, std::string group_property = "group",
	          Interface::Direction interface = Interface::NONE);
	bool with_world;
	bool cumulative;
	std::string group_property;
	Interface::Direction interface;

protected:
	double compute(const SubTrajectory& s, std::string& comment) const;
};

}  // namespace cost
}  // namespace task_constructor
}  // namespace moveit
