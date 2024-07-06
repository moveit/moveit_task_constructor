/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
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

#pragma once

#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/cost_queue.h>
#include <moveit/task_constructor/cost_terms.h>
#include <moveit/utils/moveit_error_code.h>
#include <pybind11/smart_holder.h>

/** Trampoline classes to allow inheritance in Python (overriding virtual functions) */

namespace moveit {
namespace task_constructor {

class Task;
namespace solvers {
class PlannerInterface;
}

template <class Stage = moveit::task_constructor::Stage>
class PyStage : public Stage, public pybind11::trampoline_self_life_support
{
public:
	using Stage::Stage;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		PYBIND11_OVERRIDE(void, Stage, init, robot_model);
	}
	void reset() override { PYBIND11_OVERRIDE(void, Stage, reset, ); }
};

template <class Generator = moveit::task_constructor::Generator>
class PyGenerator : public PyStage<Generator>
{
public:
	using PyStage<Generator>::PyStage;
	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, Generator, canCompute, ); }
	void compute() override { PYBIND11_OVERRIDE_PURE(void, Generator, compute, ); }
};

template <class MonitoringGenerator = moveit::task_constructor::MonitoringGenerator>
class PyMonitoringGenerator : public PyGenerator<MonitoringGenerator>
{
public:
	using PyGenerator<MonitoringGenerator>::PyGenerator;
	void onNewSolution(const SolutionBase& s) override {
		// pass solution as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, MonitoringGenerator, onNewSolution, &s);
	}
};

// Helper class to expose protected member function onNewSolution
// https://pybind11.readthedocs.io/en/stable/advanced/classes.html#binding-protected-member-functions
class PubMonitoringGenerator : public MonitoringGenerator
{
public:
	using MonitoringGenerator::onNewSolution;
};

template <class PropagatingEitherWay = moveit::task_constructor::PropagatingEitherWay>
class PyPropagatingEitherWay : public PyStage<PropagatingEitherWay>
{
public:
	using PyStage<PropagatingEitherWay>::PyStage;
	void computeForward(const InterfaceState& from_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, PropagatingEitherWay, computeForward, &from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, PropagatingEitherWay, computeBackward, &to_state);
	}
};

}  // namespace task_constructor
}  // namespace moveit

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::solvers::PlannerInterface)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::SolutionBase)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::SubTrajectory)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(ordered<moveit::task_constructor::SolutionBaseConstPtr>)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::InterfaceState)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::core::MoveItErrorCode)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::CostTerm)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::TrajectoryCostTerm)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::cost::PathLength)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::cost::DistanceToReference)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::cost::TrajectoryDuration)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::cost::LinkMotion)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::cost::Clearance)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Stage)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::PropagatingEitherWay)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::PropagatingForward)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::PropagatingBackward)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Generator)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::MonitoringGenerator)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::ContainerBase)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::SerialContainer)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::ParallelContainerBase)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Alternatives)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Fallbacks)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Merger)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::WrapperBase)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Task)
