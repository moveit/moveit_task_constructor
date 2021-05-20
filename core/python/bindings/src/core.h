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
#include <pybind11/smart_holder.h>

/** Trampoline classes to allow inheritance in Python (overriding virtual functions) */

namespace moveit {
namespace task_constructor {

class Task;
namespace solvers {
class PlannerInterface;
}

template <class T = Stage>
class PyStage : public T, public pybind11::trampoline_self_life_support
{
public:
	using T::T;

	void init(const moveit::core::RobotModelConstPtr& robot_model) override {
		PYBIND11_OVERRIDE(void, T, init, robot_model);
	}
	void reset() override { PYBIND11_OVERRIDE(void, T, reset, ); }
};

template <class T = Generator>
class PyGenerator : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	bool canCompute() const override { PYBIND11_OVERRIDE_PURE(bool, T, canCompute, ); }
	void compute() override { PYBIND11_OVERRIDE_PURE(void, T, compute, ); }
};

template <class T = MonitoringGenerator>
class PyMonitoringGenerator : public PyGenerator<T>
{
public:
	using PyGenerator<T>::PyGenerator;
	void onNewSolution(const SolutionBase& s) override {
		// pass solution as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, &s);
	}
};

class PubMonitoringGenerator : public MonitoringGenerator
{
public:
	using MonitoringGenerator::onNewSolution;
};

template <class T = PropagatingEitherWay>
class PyPropagatingEitherWay : public PyStage<T>
{
public:
	using PyStage<T>::PyStage;
	void computeForward(const InterfaceState& from_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, T, computeForward, &from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_PURE(void, T, computeBackward, &to_state);
	}
};

}  // namespace task_constructor
}  // namespace moveit

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::Property)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::PropertyMap)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::solvers::PlannerInterface)

PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::SolutionBase)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::SubTrajectory)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(ordered<moveit::task_constructor::SolutionBaseConstPtr>)
PYBIND11_SMART_HOLDER_TYPE_CASTERS(moveit::task_constructor::InterfaceState)

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
