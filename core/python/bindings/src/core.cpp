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

#include "core.h"
#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/container_p.h>
#include <moveit/task_constructor/task.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace py = pybind11;
using namespace py::literals;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

// utility function to extract index from python object
// also handles negative indexes referencing from the end
size_t convert_index(size_t size, const py::object& pyindex) {
	long index = pyindex.cast<size_t>();
	if (index < 0)
		index += size;
	if (index >= long(size) || index < 0)
		throw pybind11::index_error("Index out of range");
	return index;
}

// implement operator[](index)
template <typename T>
typename T::value_type get_item(const T& container, const py::object& index) {
	auto it = container.begin();
	std::advance(it, convert_index(container.size(), index));
	return *it;
}

py::list getForwardedProperties(const Stage& self) {
	py::list l;
	for (const std::string& value : self.forwardedProperties())
		l.append(value);
	return l;
}

void setForwardedProperties(Stage& self, const py::object& names) {
	std::set<std::string> s;
	try {
		// handle string argument as single name
		if (PyBytes_Check(names.ptr()))
			s.emplace(names.cast<std::string>());
		else  // expect iterable otherwise
			for (auto item : names)
				s.emplace(item.cast<std::string>());
	} catch (const py::cast_error& e) {
		// manually translate cast_error to type error
		PyErr_SetString(PyExc_TypeError, e.what());
		throw py::error_already_set();
	}
	self.setForwardedProperties(s);
}

}  // anonymous namespace

void export_core(pybind11::module& m) {
	/// translate InitStageException into InitStageError
	static py::exception<InitStageException> init_stage_error(m, "InitStageError");
	/// provide extended error description for InitStageException
	py::register_exception_translator([](std::exception_ptr p) {
		try {
			if (p)
				std::rethrow_exception(p);
		} catch (const InitStageException& e) {
			std::stringstream message;
			message << e;
			init_stage_error(message.str().c_str());
		}
	});

	// clang-format off
	py::classh<SolutionBase>(m, "Solution", "Solution class encapsulates a particular solution")
		.def_property("cost", &SolutionBase::cost, &SolutionBase::setCost,"float: Cost that is associated with a particular solution")
		.def_property("comment", &SolutionBase::comment, &SolutionBase::setComment, "str: Comment that is associated with a particular solution")
		.def("toMsg", [](const SolutionBasePtr& s) {
			moveit_task_constructor_msgs::Solution msg;
			s->fillMessage(msg);
			return msg;
		}, "Convert a solution object into a ros message type")
		;
	py::classh<SubTrajectory, SolutionBase>(m, "SubTrajectory", "A SubTrajectory connects interface states of compute stages")
		.def(py::init<>())
		.def_property_readonly("start", &SolutionBase::start, "InterfaceState: Start of the trajectory. Readonly property")
		.def_property_readonly("end", &SolutionBase::end, "InterfaceState: End of the trajectory. Readonly property")
		.def_property("cost", &SolutionBase::cost, &SolutionBase::setCost, "float: Cost of the solution")
		.def("markAsFailure", &SolutionBase::markAsFailure, "Mark the SubTrajectory as a failure", "msg"_a)
		.def_property_readonly("isFailure", &SolutionBase::isFailure, "bool: True if the trajectory is marked as a failure. Readonly property")
		.def_property("comment", &SolutionBase::comment, &SolutionBase::setComment, "str: Comment, which can be assigned to the trajectory")
		.def_property_readonly("markers", py::overload_cast<>(&SolutionBase::markers), R"(
			Marker_: Markers that visualize the trajectory. Readonly property.

			.. _Marker: https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
		)")
		;

	using Solutions = ordered<SolutionBaseConstPtr>;
	py::classh<Solutions>(m, "Solutions", "Encapsulates multiple solutions")
		.def("__len__", &Solutions::size)
		.def("__getitem__", &get_item<Solutions>)
		.def("__iter__", [](Solutions& self) { return py::make_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())
		;

	py::classh<InterfaceState>(m, "InterfaceState", R"(
			InterfaceState describes a potential start or goal state
			for a planning stage. A start or goal state for planning
			is essentially defined by the state of a planning scene.)")
		.def(py::init<const planning_scene::PlanningScenePtr&>(), "scene"_a)
		.def_property_readonly("properties", py::overload_cast<>(&InterfaceState::properties),
			"PropertyMap: Get access to the PropertyMap of the stage. Notice that this is a read-only property")
		.def_property_readonly("scene", &InterfaceState::scene,
			"PlanningScene: Get access to the planning scene of the interface stage. Notice that this is a read-only property")
		;

	py::classh<moveit::core::MoveItErrorCode>(m, "MoveItErrorCode", "Encapsulates moveit error code message")
		.def_readonly("val", &moveit::core::MoveItErrorCode::val, "int: Value of the error code")
		.def(PYBIND11_BOOL_ATTR, [](const moveit::core::MoveItErrorCode& err) {
			return pybind11::cast(static_cast<bool>(err));
		});

	auto stage = properties::class_<Stage, PyStage<>>(m, "Stage", R"(
		Stage base type. All other stages derive from this type.
		Object is not instantiable and should not be added to the task hierarchy
		in a standalone fashion. Rather, derive from generator or propagator stages
		to implement custom logic.
		)")
		.property<double>("timeout","float: Timeout of stage per computation")
		.property<std::string>("marker_ns", "str: Namespace for any markers that are associated to the stage")
		.def_property("forwarded_properties", getForwardedProperties, setForwardedProperties, "list: Get / set a list of forwarded properties")
		// expose name as writeable property
		.def_property("name", &Stage::name, &Stage::setName, "str: Get / set the name of the stage")
		// read-only access to properties + solutions
		.def_property_readonly("properties", py::overload_cast<>(&Stage::properties), "PropertyMap: Return the property map of the stage. Readonly property")
		.def_property_readonly("solutions", &Stage::solutions, "Solutions: Get the solutions of a stage")
		.def_property_readonly("failures", &Stage::failures, "Solutions: Get the failed compuations of a stage")
		.def("reset", &Stage::reset, "Reset the stage, clearing all solutions, interfaces and inherited properties")
		.def("init", &Stage::init, R"(
			Initialize the stage once before planning. When called, properties configured for
			initialization from parent are already defined. Push interfaces are not yet defined!
			)", "robot_model"_a);

	py::enum_<Stage::PropertyInitializerSource>(stage, "PropertyInitializerSource", R"(
		Define, from where properties should be initialized when using the `configureInitFrom()`
		functions from the PropertyMap class.
		)")
		.value("PARENT", Stage::PARENT, "Inherit properties from parent stage in task hierarchy")
		.value("INTERFACE", Stage::INTERFACE, "Inherit properties from interface, i.e. preceeding stage in the task hierarchy")
		;


	auto either_way = py::classh<PropagatingEitherWay, Stage, PyPropagatingEitherWay<>>(m, "PropagatingEitherWay", "Base class for solution forwarding in both directions")
		.def(py::init<const std::string&>(), "name"_a = std::string("PropagatingEitherWay"))
		.def("restrictDirection", &PropagatingEitherWay::restrictDirection, "Explicitly specify computation direction")
		.def("computeForward", &PropagatingEitherWay::computeForward, "Compute forward")
		.def("computeBackward", &PropagatingEitherWay::computeBackward, "Compute backward")
		//.def("sendForward", &PropagatingEitherWay::sendForward)
		//.def("sendBackward", &PropagatingEitherWay::sendBackward)
		;

	py::enum_<PropagatingEitherWay::Direction>(either_way, "Direction")
		.value("AUTO", PropagatingEitherWay::AUTO)
		.value("FORWARD", PropagatingEitherWay::FORWARD)
		.value("BACKWARD", PropagatingEitherWay::BACKWARD);

	py::classh<PropagatingForward, Stage, PyPropagatingEitherWay<PropagatingForward>>(m, "PropagatingForward", "Base class for forward solution propagation")
		.def(py::init<const std::string&>(), "name"_a = std::string("PropagatingForward"))
		;
	py::classh<PropagatingBackward, Stage, PyPropagatingEitherWay<PropagatingBackward>>(m, "PropagatingBackward", "Base class for backward solution propagation")
		.def(py::init<const std::string&>(), "name"_a = std::string("PropagatingBackward"))
		;

	properties::class_<Generator, Stage, PyGenerator<>>(m, "Generator", R"(
			Derive from this stage to implement a custom stage
			that spawns a solution. When traversing through the whole
			task hierarchy, the ``compute()`` function of this generator stage
			gets called as many times as the ``canCompute()`` returns ``true``.

			::

				class PyGenerator(core.Generator):
					""" Implements a custom 'Generator' stage with 3 compute() calls."""

					max_calls = 3

					def __init__(self, name="Generator"):
						core.Generator.__init__(self, name)
						self.reset()

					def init(self, robot_model):
						self.ps = PlanningScene(robot_model)

					def reset(self):
						core.Generator.reset(self)
						self.num = self.max_calls

					def canCompute(self):
						return self.num > 0

					def compute(self):
						self.num = self.num - 1
						self.spawn(core.InterfaceState(self.ps), self.num)

		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("generator"))
		.def("canCompute", &Generator::canCompute, "Guard for the ``compute()`` function")
		.def("compute", &Generator::compute, "Implement the stage's logic here. To interface with other stages, spawn an ``InterfaceState``")
		.def("spawn", [](Generator& self, InterfaceState& state, double cost) { self.spawn(std::move(state), cost); },
			"Spawn an interface state that gets forwarded to the next stage",
			"state"_a, "cost"_a)
		;

	properties::class_<MonitoringGenerator, Generator, PyMonitoringGenerator<>>(m, "MonitoringGenerator", R"(
		Generator that monitors solutions of another stage to make reuse of them
		Sometimes its necessary to reuse a previously planned solution, e.g. to
		traverse it in reverse order or to access the state of another generator.
		To this end, the present stage hooks into the onNewSolution() method of
		the monitored stage and forwards it to this' class onNewSolution() method.

		You may derive from this stage to implement your own stage generation logic.

		::

			class PyMonitoringGenerator(core.MonitoringGenerator):
				""" Implements a custom 'MonitoringGenerator' stage."""

				solution_multiplier = 2

				def __init__(self, name="MonitoringGenerator"):
					core.MonitoringGenerator.__init__(self, name)
					self.reset()

				def reset(self):
					core.MonitoringGenerator.reset(self)
					self.upstream_solutions = list()

				def onNewSolution(self, sol):
					self.upstream_solutions.append(sol)

				def canCompute(self):
					return bool(self.upstream_solutions)

				def compute(self):
					scene = self.upstream_solutions.pop(0).end.scene
					for i in range(self.solution_multiplier):
						self.spawn(core.InterfaceState(scene), i)

		Upon creation of the stage, assign the monitored stage:

		::

			jointspace = core.JointInterpolationPlanner()

			task = core.Task()
			current = stages.CurrentState("current")
			task.add(current)

			connect = stages.Connect(planners=[('panda_arm', jointspace)])
			task.add(connect)

			mg = PyMonitoringGenerator("generator")
			task.add(mg)

			task["generator"].setMonitoredStage(task["current"])

		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("generator"))
		.def("setMonitoredStage", &MonitoringGenerator::setMonitoredStage, "Set the reference to the Monitored Stage", "stage"_a)
		.def("_onNewSolution", &PubMonitoringGenerator::onNewSolution)
		;

	py::classh<ContainerBase, Stage>(m, "ContainerBase", "Base class for containers that implements utility functionality")
		.def("add", &ContainerBase::add, "Add a stage to the container")
		.def("insert", &ContainerBase::insert, "stage"_a, "before"_a = -1, "Insert a stage before given index into container")
		.def("remove", py::overload_cast<int>(&ContainerBase::remove), "Remove child stage by index", "pos"_a)
		.def("clear", &ContainerBase::clear, "Clear the stages of the container")
		.def("__len__", &ContainerBase::numChildren)
		.def("__getitem__", [](const ContainerBase &c, const std::string &name) -> Stage* {
			Stage* child = c.findChild(name);
			if (!child)
				throw py::index_error();
			return child;
		}, py::return_value_policy::reference_internal)
		.def("__iter__", [](const ContainerBase &c) {
			const auto& children = c.pimpl()->children();
			return py::make_iterator(children.begin(), children.end());
		}, py::keep_alive<0, 1>())  // keep container alive as long as iterator lives
		;

	py::classh<SerialContainer, ContainerBase>(m, "SerialContainer", "Base class for container containing a serial set of stages")
	    .def(py::init<const std::string&>(), "name"_a = std::string("serial container"));

	py::classh<ParallelContainerBase, ContainerBase>(m, "ParallelContainerBase", "Base class for parallel containers");

	py::classh<Alternatives, ParallelContainerBase>(m, "Alternatives", R"(
		Plan for different alternatives in parallel.
		Solution of all children are reported - sorted by cost.

		.. literalinclude:: ./../../../demo/scripts/alternatives.py
			:language: python

		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("alternatives"))
		;

	py::classh<Fallbacks, ParallelContainerBase>(m, "Fallbacks", R"(
		Plan for different alternatives in sequence.
		Try to find feasible solutions using first child.
		Only if this fails, proceed to the next child trying
		an alternative planning strategy.
		All solutions of the last active child are reported.

		.. literalinclude:: ./../../../demo/scripts/fallbacks.py
			:language: python

		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("fallbacks"))
		;

	py::classh<Merger, ParallelContainerBase>(m, "Merger", R"(
		Plan for different sub tasks in parallel and finally merge
		all sub solutions into a single trajectory

		.. literalinclude:: ./../../../demo/scripts/merger.py
			:language: python

		)")
		.def(py::init<const std::string&>(), "name"_a = std::string("merger"))
		;

	py::classh<WrapperBase, ParallelContainerBase>(m, "WrapperBase", R"(
		A wrapper wraps a single child stage, which can be
		accessed via ``wrapped()``. Implementations of this
		interface need to implement ``onNewSolution()``, which
		is called when the child has generated a new solution.
		The wrapper may reject the solution or create one or
		multiple derived solutions, potentially adapting the cost,
		as well as its start and end states.
		)")
		;

	py::classh<Task>(m, "Task", R"(
			A task is the root of a tree of stages by
			wrapping a single container
			(by default a SerialContainer),
			which serves as the root of all stages.
			See below an exampel which creates a task with a
			single stage in its hierarchy, fetching the current state
			of the planning scene.

			.. literalinclude:: ./../../../demo/scripts/current_state.py
				:language: python

		)")
		.def(py::init<const std::string&, bool>(), "ns"_a = std::string(), "introspection"_a = true)
		.def(py::init<const std::string&, bool, ContainerBase::pointer&&>(),
		     "ns"_a = std::string(), "introspection"_a = true, "container"_a)
		// read-only access to properties + solutions
		.def_property_readonly("properties", py::overload_cast<>(&Task::properties),
			"PropertyMap: Access the property map of the task")
		.def_property_readonly("solutions", &Task::solutions,
			"Solutions: Access the solutions of the task, once the sub stage hierarchy has been traversed and planned")
		.def_property_readonly("failures", &Task::failures,
			"Solutions: Inspect failures that occurred during the planning phase")
		.def_property("name", &Task::name, &Task::setName, "str: Set the name property of the task")
		.def("loadRobotModel", &Task::loadRobotModel, "robot_description"_a = "robot_description", "Load robot model from given parameter")
		.def("getRobotModel", &Task::getRobotModel)
		.def("enableIntrospection", &Task::enableIntrospection, "enabled"_a = true, "Enable introspection publish for use with `rviz`")
		.def("clear", &Task::clear, "Reset the stage hierarchy")
		.def("add", &Task::add, "Add a stage to the task hierarchy", "stage"_a)
		.def("__len__", [](const Task& t) { t.stages()->numChildren(); })
		.def("__getitem__", [](const Task& t, const std::string &name) -> Stage* {
			Stage* child = t.stages()->findChild(name);
			if (!child)
				throw py::index_error();
			return child;
		}, py::return_value_policy::reference_internal)
		.def("__iter__", [](const Task &t) {
			const auto& children = t.stages()->pimpl()->children();
			return py::make_iterator(children.begin(), children.end());
		}, py::keep_alive<0, 1>())  // keep container alive as long as iterator lives
		.def("reset", &Task::reset, "Reset all stages")
		.def("init", py::overload_cast<>(&Task::init), "Initialize all stages with a given scene")
		.def("plan", &Task::plan, "max_solutions"_a = 0, R"(
			Reset, initialize scene (if not yet done), and initialize all
			stages, then start planning with ``max_allowed_solutions``. Returns if planning
			was successful.
			)")
		.def("preempt", &Task::preempt, "Interrupt current planning (or execution)")
		.def("publish", [](Task& self, const SolutionBasePtr& solution) {
			self.introspection().publishSolution(*solution);
		}, "solution"_a, "Publish a given solution to the solution ros topic")
		.def("execute", [](const Task& self, const SolutionBasePtr& solution) {
			moveit::planning_interface::PlanningSceneInterface psi;
			moveit::planning_interface::MoveGroupInterface
				mgi(solution->start()->scene()->getRobotModel()->getJointModelGroupNames()[0]);

			moveit::planning_interface::MoveGroupInterface::Plan plan;
			moveit_task_constructor_msgs::Solution serialized;
			solution->fillMessage(serialized);

			for (const moveit_task_constructor_msgs::SubTrajectory& traj : serialized.sub_trajectory) {
				if (!traj.trajectory.joint_trajectory.points.empty()) {
					plan.trajectory_ = traj.trajectory;
					if (!mgi.execute(plan)) {
						ROS_ERROR("Execution failed! Aborting!");
						return;
					}
				}
				psi.applyPlanningScene(traj.scene_diff);
			}
			ROS_INFO("Executed successfully");
		}, "solution"_a, "Execute Solution");
	// clang-format on
}
}  // namespace python
}  // namespace moveit
