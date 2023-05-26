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
#include "utils.h"
#include <pybind11/stl.h>
#include <pybind11/functional.h>
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
		// translate cast_error to type_error with an informative message
		throw py::type_error("Expecting a string or a list of strings");
	}
	self.setForwardedProperties(s);
}

}  // anonymous namespace

void export_core(pybind11::module& m) {
	/// translate InitStageException into InitStageError
	PYBIND11_CONSTINIT static py::gil_safe_call_once_and_store<py::object> exc_storage;
	exc_storage.call_once_and_store_result([&]() { return py::exception<InitStageException>(m, "InitStageError"); });

	/// provide extended error description for InitStageException
	py::register_exception_translator([](std::exception_ptr p) {  // NOLINT(performance-unnecessary-value-param)
		try {
			if (p)
				std::rethrow_exception(p);
		} catch (const InitStageException& e) {
			std::stringstream message;
			message << e;
			py::set_error(exc_storage.get_stored(), message.str().c_str());
		}
	});

	py::classh<SolutionBase>(m, "Solution", "Abstract base class for solutions of a stage")
	    .def_property("cost", &SolutionBase::cost, &SolutionBase::setCost, "float: Cost associated with the solution")
	    .def_property("comment", &SolutionBase::comment, &SolutionBase::setComment,
	                  "str: Comment associated with the solution")
	    .def("markAsFailure", &SolutionBase::markAsFailure, "Mark the SubTrajectory as a failure", "comment"_a)
	    .def_property_readonly("isFailure", &SolutionBase::isFailure,
	                           "bool: True if the trajectory is marked as a failure (read-only)")
	    .def_property_readonly("start", &SolutionBase::start, "InterfaceState: Start of the trajectory (read-only)")
	    .def_property_readonly("end", &SolutionBase::end, "InterfaceState: End of the trajectory (read-only)")
	    .def_property_readonly(
	        "markers", py::overload_cast<>(&SolutionBase::markers),
	        ":visualization_msgs:`Marker`: Markers to visualize important aspects of the trajectory (read-only)")
	    .def(
	        "toMsg",
	        [](const SolutionBase& self) {
		        moveit_task_constructor_msgs::msg::Solution msg;
		        self.toMsg(msg);
		        return msg;
	        },
	        "Convert to the ROS message ``Solution``");

	py::classh<SubTrajectory, SolutionBase>(m, "SubTrajectory",
	                                        "Solution trajectory connecting two InterfaceStates of a stage")
	    .def(py::init<>())
	    .def_property("trajectory", &SubTrajectory::trajectory, &SubTrajectory::setTrajectory,
	                  ":moveit_msgs:`RobotTrajectory`: Actual robot trajectory");

	using Solutions = ordered<SolutionBaseConstPtr>;
	py::classh<Solutions>(m, "Solutions", "Cost-ordered list of solutions")
	    .def("__len__", &Solutions::size)
	    .def("__getitem__", &get_item<Solutions>)
	    .def(
	        "__iter__", [](Solutions& self) { return py::make_iterator(self.begin(), self.end()); },
	        py::keep_alive<0, 1>());

	py::classh<InterfaceState>(m, "InterfaceState",
	                           "Describes a potential start or goal state of a Stage. "
	                           "It comprises a PlanningScene as well as a PropertyMap.")
	    .def(py::init<const planning_scene::PlanningScenePtr&>(), "scene"_a)
	    .def_property_readonly("properties", py::overload_cast<>(&InterfaceState::properties),
	                           "PropertyMap: PropertyMap of the state (read-only).")
	    .def_property_readonly("scene", &InterfaceState::scene,
	                           "PlanningScene: PlanningScene of the state (read-only).");

	py::classh<moveit::core::MoveItErrorCode>(m, "MoveItErrorCode", "Encapsulates moveit error code message")
	    .def_readonly("val", &moveit::core::MoveItErrorCode::val, ":moveit_msgs:`MoveItErrorCodes`: error code")
	    .def(PYBIND11_BOOL_ATTR,
	         [](const moveit::core::MoveItErrorCode& err) { return pybind11::cast(static_cast<bool>(err)); });

	py::classh<CostTerm>(m, "CostTerm", "Base class for cost calculation in stages");
	auto tct = py::classh<TrajectoryCostTerm, CostTerm>(m, "TrajectoryCostTerm",
	                                                    "Base class for cost calculation of trajectories");
	py::enum_<TrajectoryCostTerm::Mode>(tct, "Mode", "Specify which states are considered for collision checking")
	    .value("AUTO", TrajectoryCostTerm::Mode::AUTO, "TRAJECTORY (if available) or START_INTERFACE")
	    .value("START_INTERFACE", TrajectoryCostTerm::Mode::START_INTERFACE, "Only consider start state")
	    .value("END_INTERFACE", TrajectoryCostTerm::Mode::END_INTERFACE, "Only consider end state")
	    .value("TRAJECTORY", TrajectoryCostTerm::Mode::TRAJECTORY, "Consider whole trajectory");

	py::classh<cost::PathLength, TrajectoryCostTerm>(m, "PathLength",
	                                                 "Computes joint-based path length along trajectory")
	    .def(py::init<>())
	    .def(py::init<std::vector<std::string>>())
	    .def(py::init<std::map<std::string, double>>());
	py::classh<cost::DistanceToReference, TrajectoryCostTerm>(m, "DistanceToReference",
	                                                          "Computes joint-based distance to reference pose")
	    .def(py::init<const moveit_msgs::msg::RobotState&, TrajectoryCostTerm::Mode, std::map<std::string, double>>(),
	         "reference"_a, "mode"_a = TrajectoryCostTerm::Mode::AUTO, "weights"_a = std::map<std::string, double>())
	    .def(py::init<const std::map<std::string, double>&, TrajectoryCostTerm::Mode, std::map<std::string, double>>(),
	         "reference"_a, "mode"_a = TrajectoryCostTerm::Mode::AUTO, "weights"_a = std::map<std::string, double>());
	py::classh<cost::TrajectoryDuration, TrajectoryCostTerm>(m, "TrajectoryDuration", "Computes duration of trajectory")
	    .def(py::init<>());
	py::classh<cost::LinkMotion, TrajectoryCostTerm>(m, "LinkMotion",
	                                                 "Computes Cartesian path length of given link along trajectory")
	    .def(py::init<std::string>(), "link_name"_a);

	py::classh<cost::Clearance, TrajectoryCostTerm>(m, "Clearance", "Computes inverse distance to collision objects")
	    .def(py::init<bool, bool, std::string, TrajectoryCostTerm::Mode>(), "with_world"_a = true,
	         "cumulative"_a = false, "group_property"_a = "group", "mode"_a = TrajectoryCostTerm::Mode::AUTO);

	auto stage =
	    properties::class_<Stage, PyStage<>>(m, "Stage", "Abstract base class of all stages.")
	        .property<double>("timeout", "float: Maximally allowed time [s] per computation step")
	        .property<std::string>("marker_ns", "str: Namespace for any markers that are associated to the stage")
	        .def_property("forwarded_properties", getForwardedProperties, setForwardedProperties,
	                      "list: set of properties forwarded from input to output InterfaceState")
	        .def_property("name", &Stage::name, &Stage::setName, "str: name of the stage displayed e.g. in rviz")
	        .def_property_readonly("properties", py::overload_cast<>(&Stage::properties),
	                               "PropertyMap: PropertyMap of the stage (read-only)")
	        .def_property_readonly("solutions", &Stage::solutions, "Successful Solutions of the stage (read-only)")
	        .def_property_readonly("failures", &Stage::failures, "Solutions: Failed Solutions of the stage (read-only)")
	        .def<void (Stage::*)(const CostTermConstPtr&)>("setCostTerm", &Stage::setCostTerm,
	                                                       "Specify a CostTerm for calculation of stage costs")
	        .def(
	            "setCostTerm", [](Stage& self, const LambdaCostTerm::SubTrajectorySignature& f) { self.setCostTerm(f); },
	            "Specify a function to calculate trajectory costs")
	        .def(
	            "setCostTerm",
	            [](Stage& self, const LambdaCostTerm::SubTrajectoryShortSignature& f) { self.setCostTerm(f); },
	            "Specify a function to calculate trajectory costs")
	        .def("reset", &Stage::reset, "Reset the Stage. Clears all solutions, interfaces and inherited properties")
	        .def("init", &Stage::init,
	             "Initialize the stage once before planning. "
	             "Will setup properties configured for initialization from parent.",
	             "robot_model"_a);

	py::enum_<Stage::PropertyInitializerSource>(
	    stage, "PropertyInitializerSource",
	    "OR-combinable flags defining a source to initialize a specific property from. "
	    "Used in :doc:`pymoveit_mtc.core.PropertyMap` ``configureInitFrom()``. ")
	    .value("PARENT", Stage::PARENT, "Inherit properties from parent stage")
	    .value("INTERFACE", Stage::INTERFACE, "Inherit properties from the input InterfaceState");

	auto either_way = py::classh<PropagatingEitherWay, Stage, PyPropagatingEitherWay<>>(
	                      m, "PropagatingEitherWay", "Base class for propagator-like stages")
	                      .def(py::init<const std::string&>(), "name"_a = std::string("PropagatingEitherWay"))
	                      .def("restrictDirection", &PropagatingEitherWay::restrictDirection,
	                           "Explicitly specify computation direction")
	                      .def("computeForward", &PropagatingEitherWay::computeForward, "Compute forward")
	                      .def("computeBackward", &PropagatingEitherWay::computeBackward, "Compute backward")
	    //.def("sendForward", &PropagatingEitherWay::sendForward)
	    //.def("sendBackward", &PropagatingEitherWay::sendBackward)
	    ;

	py::enum_<PropagatingEitherWay::Direction>(either_way, "Direction", "Propagation direction")
	    .value("AUTO", PropagatingEitherWay::AUTO)
	    .value("FORWARD", PropagatingEitherWay::FORWARD, "Propagating forwards from start to end")
	    .value("BACKWARD", PropagatingEitherWay::BACKWARD, "Propagating backwards from end to start");

	py::classh<PropagatingForward, Stage, PyPropagatingEitherWay<PropagatingForward>>(
	    m, "PropagatingForward", "Base class for forward-propagating stages")
	    .def(py::init<const std::string&>(), "name"_a = std::string("PropagatingForward"));
	py::classh<PropagatingBackward, Stage, PyPropagatingEitherWay<PropagatingBackward>>(
	    m, "PropagatingBackward", "Base class for backward-propagating stages")
	    .def(py::init<const std::string&>(), "name"_a = std::string("PropagatingBackward"));

	properties::class_<Generator, Stage, PyGenerator<>>(m, "Generator", R"(
			Base class for generator-like stages

			Derive from this stage to implement a custom generator stage that can produce new seed states w/o prior knowledge.
			Implement the virtual methods as follows::

				class MyGenerator(core.Generator):
					"""Implements a custom 'Generator' stage that produces maximally 3 solutions."""

					def __init__(self, name="Generator"):
						core.Generator.__init__(self, name)
						self.reset()

					def init(self, robot_model):
						self.ps = PlanningScene(robot_model)

					def reset(self):
						core.Generator.reset(self)

					def canCompute(self):
						return len(self.solutions) < 3  # maximally produce 3 solutions

					def compute(self):
						self.spawn(core.InterfaceState(self.ps), cost=len(self.solutions))
		)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("Generator"))
	    .def("canCompute", &Generator::canCompute, "Return ``True`` if the stage can still produce solutions.")
	    .def("compute", &Generator::compute, "Compute an actual solution and ``spawn`` an ``InterfaceState``")
	    .def(
	        "spawn", [](Generator& self, InterfaceState& state, double cost) { self.spawn(std::move(state), cost); },
	        "Spawn an ``InterfaceState`` to both, start and end interface", "state"_a, "cost"_a);

	properties::class_<MonitoringGenerator, Generator, PyMonitoringGenerator<>>(m, "MonitoringGenerator", R"(
			Base class for monitoring generator stages

			To implement a generator stage that draws on some previously computed solution, you need to derive
			from ``MonitoringGenerator`` - monitoring the solutions produced by another stage.
			Each time, the monitored stage produces a new solution, the method ``onNewSolution()`` of the
			MonitoringGenerator is called. Usually, you schedule this solution for later processing in ``compute()``::

				class PyMonitoringGenerator(core.MonitoringGenerator):
					""" Implements a custom 'MonitoringGenerator' stage."""

					solution_multiplier = 2

					def __init__(self, name="MonitoringGenerator"):
						core.MonitoringGenerator.__init__(self, name)
						self.reset()

					def reset(self):
						core.MonitoringGenerator.reset(self)
						self.pending = []

					def onNewSolution(self, sol):
						self.pending.append(sol)

					def canCompute(self):
						return bool(self.pending)

					def compute(self):
						# fetch first pending upstream solution ...
						scene = self.pending.pop(0).end.scene
						# ... and generate new solutions derived from it
						for i in range(self.solution_multiplier):
							self.spawn(core.InterfaceState(scene), i)

			Upon creation of the stage, assign the monitored stage as follows::

				jointspace = core.JointInterpolationPlanner()

				task = core.Task()
				current = stages.CurrentState("current")
				task.add(current)

				connect = stages.Connect(planners=[('panda_arm', jointspace)])
				task.add(connect)

				mg = PyMonitoringGenerator("generator")
				mg.setMonitoredStage(task["current"])
				task.add(mg)
			)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("generator"))
	    .def("setMonitoredStage", &MonitoringGenerator::setMonitoredStage, "Set the monitored ``Stage``", "stage"_a)
	    .def("_onNewSolution", &PubMonitoringGenerator::onNewSolution);

	py::classh<ContainerBase, Stage>(m, "ContainerBase", R"(
			Abstract base class for container stages
			Containers allow encapsulation and reuse of planning functionality in a hierachical fashion.
			You can iterate of the children of a container and access them by name.)")
	    .def(
	        "add",
	        [](ContainerBase& c, const py::args& args) {
		        for (auto it = args.begin(), end = args.end(); it != end; ++it)
			        c.add(it->cast<Stage::pointer>());
	        },
	        "Insert a stage at the end of the current children list")
	    .def("insert", &ContainerBase::insert, "stage"_a, "before"_a = -1,
	         "Insert a stage before the given index into the children list")
	    .def("remove", py::overload_cast<int>(&ContainerBase::remove), "Remove child stage by index", "pos"_a)
	    .def("remove", py::overload_cast<Stage*>(&ContainerBase::remove), "Remove child stage by instance", "child"_a)
	    .def("clear", &ContainerBase::clear, "Remove all stages from the container")
	    .def("__len__", &ContainerBase::numChildren)
	    .def(
	        "__getitem__",
	        [](const ContainerBase& c, const std::string& name) -> Stage* {
		        Stage* child = c.findChild(name);
		        if (!child)
			        throw py::index_error();
		        return child;
	        },
	        py::return_value_policy::reference_internal)
	    .def(
	        "__getitem__",
	        [](const ContainerBase& c, int idx) -> Stage* {
		        Stage* child = c[idx];
		        if (!child)
			        throw py::index_error();
		        return child;
	        },
	        py::return_value_policy::reference_internal)
	    .def(
	        "__iter__",
	        [](const ContainerBase& c) {
		        const auto& children = c.pimpl()->children();
		        return py::make_iterator(children.begin(), children.end());
	        },
	        py::keep_alive<0, 1>())  // keep container alive as long as iterator lives
	    ;

	py::classh<SerialContainer, ContainerBase>(m, "SerialContainer", "Container implementing a linear planning sequence")
	    .def(py::init<const std::string&>(), "name"_a = std::string("SerialContainer"));

	py::classh<ParallelContainerBase, ContainerBase>(m, "ParallelContainerBase",
	                                                 "Abstract base class for parallel containers");

	py::classh<Alternatives, ParallelContainerBase>(m, "Alternatives", R"(
			Plan for different alternatives in parallel.
			Solutions of all children are considered simultaneously.
			See :ref:`How-To-Guides <subsubsec-howto-alternatives>` for an example.
			)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("Alternatives"));

	py::classh<Fallbacks, ParallelContainerBase>(m, "Fallbacks", R"(
			Plan for different alternatives in sequence
			Try to find feasible solutions using the children in sequence. The behaviour slightly differs for the indivual stage types:

			- Generator: Proceed to next child if currently active one exhausted its solution, i.e. returns ``canCompute() == False``.
			- Propagator: Forward an incoming ``InterfaceState`` to the next child if the current one ultimately failed on it.
			- Connect: Only ``Connect`` stages are supported. Pairs of ``InterfaceStates`` are forward to the next child on failure of the current child.

			See :ref:`How-To-Guides <subsubsec-howto-fallbacks>` for an example.
			)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("Fallbacks"));

	py::classh<Merger, ParallelContainerBase>(m, "Merger", R"(
			Plan for different sub tasks in parallel and eventually merge all sub solutions into a single trajectory
			This requires all children to operate on disjoint ``JointModelGroups``.

			See :ref:`How-To-Guides <subsubsec-howto-merger>` for an example.
			)")
	    .def(py::init<const std::string&>(), "name"_a = std::string("merger"));

	py::classh<WrapperBase, ParallelContainerBase>(m, "WrapperBase", R"(
			Base class for wrapping containers, which can be used to filter or modify solutions generated by the single child.
			Implementations of this interface need to implement ``onNewSolution()`` to process a solution generated by the child.
			The wrapper may reject the solution or create one or multiple derived solutions, potentially adapting the cost,
			the trajectory and output ``InterfaceStates``.
			)");

	py::classh<Task>(m, "Task", R"(Root stage of a planning pipeline.
			A task stage usually wraps a single container (by default ``SerialContainer``) stage.
			The class provides methods to ``plan()`` for the configured pipeline and retrieve full solutions.)")
	    .def(py::init<const std::string&, bool>(), "ns"_a = std::string(), "introspection"_a = true)
	    .def(py::init<const std::string&, bool, ContainerBase::pointer&&>(), "ns"_a = std::string(),
	         "introspection"_a = true, "container"_a)

	    .def_property_readonly("properties", py::overload_cast<>(&Task::properties),
	                           "PropertyMap: PropertyMap of the stage (read-only)")
	    .def_property_readonly("solutions", &Task::solutions, "Successful Solutions of the stage (read-only)")
	    .def_property_readonly("failures", &Task::failures, "Solutions: Failed Solutions of the stage (read-only)")
	    .def_property("name", &Task::name, &Task::setName, "str: name of the task displayed e.g. in rviz")

	    .def("loadRobotModel", &Task::loadRobotModel, "node"_a, "robot_description"_a = "robot_description",
	         "Load robot model from given ROS parameter")
	    .def("getRobotModel", &Task::getRobotModel)
	    .def("enableIntrospection", &Task::enableIntrospection, "enabled"_a = true,
	         "Enable publishing intermediate results for inspection in ``rviz``")
	    .def("clear", &Task::clear, "Reset the stage task (and all its stages)")
	    .def(
	        "add",
	        [](Task& t, const py::args& args) {
		        for (auto it = args.begin(), end = args.end(); it != end; ++it)
			        t.add(it->cast<Stage::pointer>());
	        },
	        "Append stage(s) to the task's top-level container")
	    .def("insert", &Task::insert, "stage"_a, "before"_a = -1, "Insert stage before given index")
	    .def("__len__", [](const Task& t) { t.stages()->numChildren(); })
	    .def(
	        "__getitem__",
	        [](const Task& t, const std::string& name) -> Stage* {
		        Stage* child = t.stages()->findChild(name);
		        if (!child)
			        throw py::index_error();
		        return child;
	        },
	        py::return_value_policy::reference_internal)
	    .def(
	        "__getitem__",
	        [](const Task& t, int idx) -> Stage* {
		        Stage* child = t.stages()->operator[](idx);
		        if (!child)
			        throw py::index_error();
		        return child;
	        },
	        py::return_value_policy::reference_internal)
	    .def(
	        "__iter__",
	        [](const Task& t) {
		        const auto& children = t.stages()->pimpl()->children();
		        return py::make_iterator(children.begin(), children.end());
	        },
	        py::keep_alive<0, 1>())  // keep container alive as long as iterator lives
	    .def(
	        "setCostTerm", [](Task& self, const CostTermConstPtr& c) { self.setCostTerm(c); },
	        "Specify a CostTerm for calculation of stage costs")
	    .def(
	        "setCostTerm", [](Task& self, const LambdaCostTerm::SubTrajectorySignature& f) { self.setCostTerm(f); },
	        "Specify a function to calculate trajectory costs")
	    .def(
	        "setCostTerm", [](Task& self, const LambdaCostTerm::SubTrajectoryShortSignature& f) { self.setCostTerm(f); },
	        "Specify a function to calculate trajectory costs")
	    .def("reset", &Task::reset, "Reset task (and all its stages)")
	    .def("init", py::overload_cast<>(&Task::init), "Initialize the task (and all its stages)")
	    .def("plan", &Task::plan, "max_solutions"_a = 0, R"(
			Reset, init, and plan. Planning is limited to ``max_allowed_solutions``.
			Returns if planning was successful.)")
	    .def("preempt", &Task::preempt, "Interrupt current planning (or execution)")
	    .def(
	        "publish",
	        [](Task& self, const SolutionBasePtr& solution) { self.introspection().publishSolution(*solution); },
	        "solution"_a, "Publish the given solution to the ROS topic ``solution``")
	    .def("execute", &Task::execute, "solution"_a, "Send given solution to ``move_group`` node for execution");
}
}  // namespace python
}  // namespace moveit
