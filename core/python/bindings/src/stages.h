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

#include "core.h"

#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/stages/move_to.h>

/** Trampoline classes to allow inheritance in Python (overriding virtual functions) */

namespace moveit {
namespace task_constructor {
namespace stages {

template <class T = MoveTo>
class PyMoveTo : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
	void computeForward(const InterfaceState& from_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_IMPL(void, T, "computeForward", &from_state);
		return T::computeForward(from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_IMPL(void, T, "computeBackward", &to_state);
		return T::computeBackward(to_state);
	}
};

template <class T = MoveRelative>
class PyMoveRelative : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
	void computeForward(const InterfaceState& from_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_IMPL(void, T, "computeForward", &from_state);
		return T::computeForward(from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		// pass InterfaceState as pointer to trigger passing by reference
		PYBIND11_OVERRIDE_IMPL(void, T, "computeBackward", &to_state);
		return T::computeBackward(to_state);
	}
};

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
