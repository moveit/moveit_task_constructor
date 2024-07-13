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
#include <moveit_msgs/msg/robot_state.h>

namespace moveit {
namespace task_constructor {

/** basic interface to compute costs for solutions
 *
 * If your cost term will only work on SubTrajectory solution objects,
 * inherit from TrajectoryCostTerm instead.
 */

MOVEIT_CLASS_FORWARD(CostTerm);
class CostTerm
{
public:
	CostTerm() = default;
	CostTerm(std::nullptr_t) : CostTerm{} {}
	virtual ~CostTerm() = default;

	virtual double operator()(const SubTrajectory& s, std::string& comment) const;
	virtual double operator()(const SolutionSequence& s, std::string& comment) const;
	virtual double operator()(const WrappedSolution& s, std::string& comment) const;
};

/** base class for cost terms that only work on SubTrajectory solutions
 *
 */
class TrajectoryCostTerm : public CostTerm
{
public:
	enum class Mode
	{
		AUTO /* TRAJECTORY, or START_INTERFACE if no trajectory is given */,
		START_INTERFACE,
		END_INTERFACE,
		TRAJECTORY
	};

	double operator()(const SolutionSequence& s, std::string& comment) const override;
	double operator()(const WrappedSolution& s, std::string& comment) const override;
};

class LambdaCostTerm : public TrajectoryCostTerm
{
public:
	using SubTrajectorySignature = std::function<double(const SubTrajectory&, std::string&)>;
	using SubTrajectoryShortSignature = std::function<double(const SubTrajectory&)>;

	// accept lambdas according to either signature above
	template <typename Term, typename Signature = decltype(signatureMatcher(std::declval<Term>()))>
	LambdaCostTerm(const Term& t) : LambdaCostTerm{ Signature{ t } } {}

	LambdaCostTerm(const SubTrajectorySignature& term);
	LambdaCostTerm(const SubTrajectoryShortSignature& term);

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

protected:
	SubTrajectorySignature term_;

private:
	template <typename T>
	static auto signatureMatcher(const T& t) -> decltype(t(SubTrajectory{}), SubTrajectoryShortSignature{});
	template <typename T>
	static auto signatureMatcher(const T& t) -> decltype(t(SubTrajectory{}, std::string{}), SubTrajectorySignature{});
};

namespace cost {

/// add a constant cost to each solution
class Constant : public CostTerm
{
public:
	Constant(double c) : cost{ c } {};

	double operator()(const SubTrajectory& s, std::string& comment) const override;
	double operator()(const SolutionSequence& s, std::string& comment) const override;
	double operator()(const WrappedSolution& s, std::string& comment) const override;

	double cost;
};

/// trajectory length with optional weighting for different joints
class PathLength : public TrajectoryCostTerm
{
public:
	/// By default, all joints are considered with same weight of 1.0
	PathLength() = default;
	/// Limit measurements to given joint names
	PathLength(std::vector<std::string> joints);
	/// Limit measurements to given joints and use given weighting
	PathLength(std::map<std::string, double> j) : joints(std::move(j)) {}

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

	std::map<std::string, double> joints;  //< joint weights
};

/// (weighted) joint-space distance to reference pose
class DistanceToReference : public TrajectoryCostTerm
{
public:
	DistanceToReference(const moveit_msgs::msg::RobotState& ref, Mode m = Mode::AUTO,
	                    std::map<std::string, double> w = std::map<std::string, double>());
	DistanceToReference(const std::map<std::string, double>& ref, Mode m = Mode::AUTO,
	                    std::map<std::string, double> w = std::map<std::string, double>());

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;

	moveit_msgs::msg::RobotState reference;
	std::map<std::string, double> weights;
	Mode mode;
};

/// execution duration of the whole trajectory
class TrajectoryDuration : public TrajectoryCostTerm
{
public:
	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

/** length of Cartesian trajection of a link */
class LinkMotion : public TrajectoryCostTerm
{
public:
	LinkMotion(std::string link_name);

	std::string link_name;

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

/** inverse distance to collision
 *
 * \arg with_world check distances to world objects or look at self-collisions
 * \arg cumulative if true, compute clearance as aggregated distance of all bodies
 * \arg group_property the name of the property which defines the group to look at
 * \arg interface compute distances using START or END interface of solution *only*, instead of averaging over
 * trajectory
 * */
class Clearance : public TrajectoryCostTerm
{
public:
	Clearance(bool with_world = true, bool cumulative = false, std::string group_property = "group",
	          Mode mode = Mode::AUTO);
	bool with_world;
	bool cumulative;
	std::string group_property;

	Mode mode;

	std::function<double(double)> distance_to_cost;

	using TrajectoryCostTerm::operator();
	double operator()(const SubTrajectory& s, std::string& comment) const override;
};

}  // namespace cost
}  // namespace task_constructor
}  // namespace moveit
