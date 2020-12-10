#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <pybind11/pybind11.h>

/** Trampoline classes to allow inheritance in Python (overriding virtual functions) */

namespace moveit {
namespace task_constructor {

template <class T = Stage>
class PyStage : public T
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
	void onNewSolution(const SolutionBase& s) override { PYBIND11_OVERRIDE_PURE(void, T, onNewSolution, s); }
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
		PYBIND11_OVERRIDE_PURE(void, T, computeForward, from_state);
	}
	void computeBackward(const InterfaceState& to_state) override {
		PYBIND11_OVERRIDE_PURE(void, T, computeBackward, to_state);
	}
};

}  // namespace task_constructor
}  // namespace moveit
