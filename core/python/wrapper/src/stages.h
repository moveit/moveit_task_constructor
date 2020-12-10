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
		PYBIND11_OVERRIDE(void, T, computeForward, from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		PYBIND11_OVERRIDE(void, T, computeBackward, to_state);
	}
};

template <class T = MoveRelative>
class PyMoveRelative : public PyPropagatingEitherWay<T>
{
public:
	using PyPropagatingEitherWay<T>::PyPropagatingEitherWay;
	void computeForward(const InterfaceState& from_state) override {
		PYBIND11_OVERRIDE(void, T, computeForward, from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		PYBIND11_OVERRIDE(void, T, computeBackward, to_state);
	}
};

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
