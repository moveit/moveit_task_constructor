/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Hamburg University
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
 *   * Neither the name of Bielefeld University nor the names of its
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
/* Authors: Michael Goerner */

#include <moveit/task_constructor/stages/predicate_filter.h>

#include <moveit/task_constructor/storage.h>

#include <moveit/planning_scene/planning_scene.h>

#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <functional>

namespace moveit {
namespace task_constructor {
namespace stages {

PredicateFilter::PredicateFilter(const std::string& name, Stage::pointer&& child)
  : WrapperBase(name, std::move(child)) {
	auto& p = properties();
	p.declare<Predicate>("predicate", "predicate to filter wrapped solutions");
	p.declare<bool>("ignore_filter", false, "ignore predicate and forward all solutions");
}

void PredicateFilter::init(const moveit::core::RobotModelConstPtr& robot_model) {
	InitStageException errors;

	try {
		WrapperBase::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// In theory this could be set in interface states
	// but we enforce it here to keep code flow sane and maintainable
	if (props.get("predicate").empty()) {
		InitStageException e(*this, "predicate is not specified");
		errors.append(e);
	}

	if (errors)
		throw errors;
}

void PredicateFilter::onNewSolution(const SolutionBase& s) {
	const auto& props = properties();

	// false-positive in clang-tidy 10.0.0: predicate might change comment
	// NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
	std::string comment = s.comment();

	double cost = s.cost();
	if (!props.get<bool>("ignore_filter") && !props.get<Predicate>("predicate")(s, comment))
		cost = std::numeric_limits<double>::infinity();

	liftSolution(s, cost, comment);
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
