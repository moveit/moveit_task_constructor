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
	py::classh<SolutionBase>(m, "Solution", R"pbdoc(
		Solution class encapsulates a particular solution.
		)pbdoc")
		.def_property("cost", &SolutionBase::cost, &SolutionBase::setCost, R"pbdoc(
			float: Cost that is associated with a particular solution.
		)pbdoc")
		.def_property("comment", &SolutionBase::comment, &SolutionBase::setComment, R"pbdoc(
			str: Comment that is associated with a particular solution.
		)pbdoc")
		.def("toMsg", [](const SolutionBasePtr& s) {
			moveit_task_constructor_msgs::Solution msg;
			s->fillMessage(msg);
			return msg;
		}, R"pbdoc(
			toMsg()

			Args:
				None
			Returns:
				ROS message type of the solution.

			Convert a solution object into a ros message type.
		)pbdoc")
		;
	py::classh<SubTrajectory, SolutionBase>(m, "SubTrajectory", R"pbdoc(
		SubTrajectory()

		A SubTrajectory connects interface states of compute stages.

		Args:
			None
		)pbdoc")
		.def(py::init<>())
		.def_property_readonly("start", &SolutionBase::start, R"pbdoc(
			InterfaceState: Start of the trajectory. Readonly property.
		)pbdoc")
		.def_property_readonly("end", &SolutionBase::end, R"pbdoc(
			InterfaceState: End of the trajectory. Readonly property.
		)pbdoc")
		.def_property("cost", &SolutionBase::cost, &SolutionBase::setCost, R"pbdoc(
			float: Cost of the solution.
		)pbdoc")
		.def("markAsFailure", &SolutionBase::markAsFailure, R"pbdoc(
			markAsFailure(msg)

			Args:
				msg (str): Failure message.
			Returns:
				None

			Mark the SubTrajectory as a failure.
		)pbdoc")
		.def_property_readonly("isFailure", &SolutionBase::isFailure, R"pbdoc(
			bool: True if the trajectory is marked as a failure. Readonly property.
		)pbdoc")
		.def_property("comment", &SolutionBase::comment, &SolutionBase::setComment, R"pbdoc(
			str: Comment, which can be assigned to the trajectory.
		)pbdoc")
		.def_property_readonly("markers", py::overload_cast<>(&SolutionBase::markers), R"pbdoc(
			Marker_: Markers that visualize the trajectory. Readonly property.

			.. _Marker: https://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html
		)pbdoc")
		;

	using Solutions = ordered<SolutionBaseConstPtr>;
	py::classh<Solutions>(m, "Solutions")
		.def("__len__", &Solutions::size)
		.def("__getitem__", &get_item<Solutions>)
		.def("__iter__", [](Solutions& self) { return py::make_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())
		;

	py::classh<InterfaceState>(m, "InterfaceState", R"pbdoc(
		InterfaceState(scene)

		Args:
			scene (obj): Desired Planning Scene at the interface.

		InterfaceState describes a potential start or goal state
		for a planning stage. A start or goal state for planning
		is essentially defined by the state of a planning scene.

	)pbdoc")
		.def(py::init<const planning_scene::PlanningScenePtr&>(), py::arg("scene"))
		.def_property_readonly("properties", py::overload_cast<>(&InterfaceState::properties), R"pbdoc(
			PropertyMap: Get access to the PropertyMap of the stage.
				Notice that this is a read-only property.
		)pbdoc")
		.def_property_readonly("scene", &InterfaceState::scene, R"pbdoc(
			PlanningScene: Get access to the planning scene of the interface stage.
				Notice that this is a read-only property.
		)pbdoc")
		;

	py::classh<moveit::core::MoveItErrorCode>(m, "MoveItErrorCode")
		.def_readonly("val", &moveit::core::MoveItErrorCode::val)
		.def(PYBIND11_BOOL_ATTR, [](const moveit::core::MoveItErrorCode& err) {
			return pybind11::cast(static_cast<bool>(err));
		});

	auto stage = properties::class_<Stage, PyStage<>>(m, "Stage", R"pbdoc(
		Stage base type. All other stages derive from this type.
		Object is not instantiable and should not be added to the task hierarchy
		in a standalone fashion. Rather, derive from generator or propagator stages
		to implement custom logic.
		)pbdoc")
		.property<double>("timeout", R"pbdoc(
			float: Timeout of stage per computation.
		)pbdoc")
		.property<std::string>("marker_ns", R"pbdoc(
			str: Namespace for any markers that are associated to the stage.
		)pbdoc")
		.def_property("forwarded_properties", getForwardedProperties, setForwardedProperties, R"pbdoc(
			list: Get / set a list of forwarded properties.
		)pbdoc")
		// expose name as writeable property
		.def_property("name", &Stage::name, &Stage::setName, R"pbdoc(
			str: Get / set the name of the stage.
		)pbdoc")
		// read-only access to properties + solutions
		.def_property_readonly("properties", py::overload_cast<>(&Stage::properties), R"pbdoc(
			PropertyMap: Return the property map of the stage. Readonly property.
		)pbdoc")
		.def_property_readonly("solutions", &Stage::solutions, R"pbdoc(
			Solutions: Get the solutions of a stage.
		)pbdoc")
		.def_property_readonly("failures", &Stage::failures, R"pbdoc(
			Solutions: Get the failed compuations of a stage.
		)pbdoc")
		.def("reset", &Stage::reset, R"pbdoc(
			reset()

			Args:
				None
			Returns:
				None

				Reset the stage, clearing all solutions, interfaces and inherited properties.
		)pbdoc")
		.def("init", &Stage::init, R"pbdoc(
			init(robot_model)

			Args:
				robot_model (RobotModel): Initialize the stage with a particular robot model.
			Returns:
				None

			Initialize the stage once before planning.
			When called, properties configured for initialization from parent are already defined.
			Push interfaces are not yet defined!
		)pbdoc");

	py::enum_<Stage::PropertyInitializerSource>(stage, "PropertyInitializerSource", R"pbdoc(
		Define, from where properties should be initialized when using the `configureInitFrom()`
		functions from the PropertyMap class.
		)pbdoc")
		.value("PARENT", Stage::PARENT, R"pbdoc(
			Inherit properties from parent stage in task hierarchy.
		)pbdoc")
		.value("INTERFACE", Stage::INTERFACE, R"pbdoc(
			Inherit properties from interface, i.e. preceeding stage
			in the task hierarchy.
		)pbdoc")
		;


	auto either_way = py::classh<PropagatingEitherWay, Stage, PyPropagatingEitherWay<>>(m, "PropagatingEitherWay")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingEitherWay"))
		.def("restrictDirection", &PropagatingEitherWay::restrictDirection)
		.def("computeForward", &PropagatingEitherWay::computeForward, "compute forward")
		.def("computeBackward", &PropagatingEitherWay::computeBackward, "compute backward")
		//.def("sendForward", &PropagatingEitherWay::sendForward)
		//.def("sendBackward", &PropagatingEitherWay::sendBackward)
		;

	py::enum_<PropagatingEitherWay::Direction>(either_way, "Direction")
		.value("AUTO", PropagatingEitherWay::AUTO)
		.value("FORWARD", PropagatingEitherWay::FORWARD)
		.value("BACKWARD", PropagatingEitherWay::BACKWARD);

	py::classh<PropagatingForward, Stage, PyPropagatingEitherWay<PropagatingForward>>(m, "PropagatingForward")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingForward"))
		;
	py::classh<PropagatingBackward, Stage, PyPropagatingEitherWay<PropagatingBackward>>(m, "PropagatingBackward")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingBackward"))
		;

	properties::class_<Generator, Stage, PyGenerator<>>(m, "Generator", R"pbdoc(
			Generator(name)

			Args:
				name (str): Name of the stage.

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

		)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("generator"))
		.def("canCompute", &Generator::canCompute, R"pbdoc(
			canCompute()

			Args:
				None
			Returns:
				bool: Should ``compute()`` be called?

			Guard for the ``compute()`` function.
		)pbdoc")
		.def("compute", &Generator::compute, R"pbdoc(
			compute()

			Args:
				None
			Returns:
				None

			Implement the stage's logic here. To interface with other stages,
			spawn an ``InterfaceState``.
		)pbdoc")
		.def("spawn", [](Generator& self, InterfaceState& state, double cost) { self.spawn(std::move(state), cost); }, R"pbdoc(
			spawn(state, cost)

			Args:
				state (InterfaceState): The Interface State that should be used.
				cost (float): The cost associated with that solution.
			Returns:
				None

			Spawn an interface state that gets forwarded to the next stage.
		)pbdoc")
		;

	properties::class_<MonitoringGenerator, Generator, PyMonitoringGenerator<>>(m, "MonitoringGenerator", R"pbdoc(
		MonitoringGenerator(name)

		Args:
			name (str): Name of the stage.

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

		)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("generator"))
		.def("setMonitoredStage", &MonitoringGenerator::setMonitoredStage, R"pbdoc(
			setMonitoredStage(stage)

			Args:
				stage (Stage): Monitor solutions of this stage.
			Returns:
				None

			Set the reference to the Monitored Stage.
		)pbdoc")
		.def("_onNewSolution", &PubMonitoringGenerator::onNewSolution)
		;

	py::classh<ContainerBase, Stage>(m, "ContainerBase")
		.def("add", &ContainerBase::add)
		.def("insert", &ContainerBase::insert, py::arg("stage"), py::arg("before") = -1)
		.def("remove", py::overload_cast<int>(&ContainerBase::remove), R"pbdoc(
			Remove child stage by index.
		)pbdoc")
		.def("clear", &ContainerBase::clear)
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

	py::classh<SerialContainer, ContainerBase>(m, "SerialContainer")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("serial container"));

	py::classh<ParallelContainerBase, ContainerBase>(m, "ParallelContainerBase");

	py::classh<Alternatives, ParallelContainerBase>(m, "Alternatives", R"pbdoc(
		Alternatives(name)

		Args:
			name (str): Name of the stage.

		Plan for different alternatives in parallel.
		Solution of all children are reported - sorted by cost.

		.. literalinclude:: ./../../../demo/scripts/alternatives.py
			:language: python

	)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("alternatives"));

	py::classh<Fallbacks, ParallelContainerBase>(m, "Fallbacks", R"pbdoc(
		Fallbacks(name)

		Args:
			name (str): Name of the stage.

		Plan for different alternatives in sequence.
		Try to find feasible solutions using first child.
		Only if this fails, proceed to the next child trying
		an alternative planning strategy.
		All solutions of the last active child are reported.

		.. literalinclude:: ./../../../demo/scripts/fallbacks.py
			:language: python

	)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("fallbacks"));

	py::classh<Merger, ParallelContainerBase>(m, "Merger", R"pbdoc(
		Merger(name)

		Args:
			name (str): Name of the stage.

		Plan for different sub tasks in parallel and finally merge
		all sub solutions into a single trajectory

		.. literalinclude:: ./../../../demo/scripts/merger.py
			:language: python

	)pbdoc")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("merger"));

	py::classh<WrapperBase, ParallelContainerBase>(m, "WrapperBase", R"pbdoc(
		WrapperBase(name, child)

		Args:
			name (str): Name of the stage.
			child (Stage): Wrapped stage.

		A wrapper wraps a single child stage, which can be
		accessed via ``wrapped()``. Implementations of this
		interface need to implement ``onNewSolution()``, which
		is called when the child has generated a new solution.
		The wrapper may reject the solution or create one or
		multiple derived solutions, potentially adapting the cost,
		as well as its start and end states.
	)pbdoc");

	py::classh<Task>(m, "Task", R"pbdoc(
			Task(ns, introspection)

			Args:
				ns (str): Namespace of the task.
				introspection (bool): Enable introspection.

			A task is the root of a tree of stages by
			wrapping a single container
			(by default a SerialContainer),
			which serves as the root of all stages.
			See below an exampel which creates a task with a
			single stage in its hierarchy, fetching the current state
			of the planning scene.

			.. literalinclude:: ./../../../demo/scripts/current_state.py
				:language: python

		)pbdoc")
		.def(py::init<const std::string&, bool>(), py::arg("ns") = std::string(), py::arg("introspection") = true)
		.def(py::init<const std::string&, bool, ContainerBase::pointer&&>(),
		     py::arg("ns") = std::string(), py::arg("introspection") = true, py::arg("container"))
		// read-only access to properties + solutions
		.def_property_readonly("properties", py::overload_cast<>(&Task::properties), R"pbdoc(
			PropertyMap: Access the property map of the task.
		)pbdoc")
		.def_property_readonly("solutions", &Task::solutions, R"pbdoc(
			Solutions: Access the solutions of the task, once the sub stage hierarchy has been
			traversed and planned.
		)pbdoc")
		.def_property_readonly("failures", &Task::failures, R"pbdoc(
			Solutions: Inspect failures that occurred during the planning phase.
		)pbdoc")
		.def_property("name", &Task::name, &Task::setName, R"pbdoc(
			str: Set the name property of the task.
		)pbdoc")
		.def("loadRobotModel", &Task::loadRobotModel, py::arg("robot_description") = "robot_description",
			R"pbdoc(
			loadRobotModel(robot_description)

			Args:
				robot_description (str): Which robot model to load.
			Returns:
				None

			Load robot model from given parameter.
			)pbdoc")
		.def("getRobotModel", &Task::getRobotModel)
		.def("enableIntrospection", &Task::enableIntrospection, py::arg("enabled") = true, R"pbdoc(
			enableIntrospection(enable)

			Args:
				enable (bool): Defaults to true.
			Returns:
				None

			Enable introspection publish for use with `rviz`.
			)pbdoc")
		.def("clear", &Task::clear, R"pbdoc(
			clear()

			Args:
				None
			Returns:
				None

			Reset the stage hierarchy.
			)pbdoc")
		.def("add", &Task::add, R"pbdoc(
			add(stage)

			Args:
				stage (Stage): Stage to be added to the task hierarchy.
			Returns:
				None

			Add a stage to the task hierarchy.
			)pbdoc")
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
		.def("reset", &Task::reset, R"pbdoc(
			reset()

			Args:
				None
			Returns:
				None

			Reset all stages.
			)pbdoc")
		.def("init", py::overload_cast<>(&Task::init), R"pbdoc(
			init()

			Args:
				None
			Returns:
				None

			Initialize all stages with a given scene.
		)pbdoc")
		.def("plan", &Task::plan, py::arg("max_solutions") = 0, R"pbdoc(
			plan(max_solutions)

			Args:
				max_solutions (int): Maximum allowed solutions of the planning process.
			Returns:
				Was the planning successful?

			Reset, initialize scene (if not yet done), and initialize all
			stages, then start planning.
			)pbdoc")
		.def("preempt", &Task::preempt, R"pbdoc(
			preempt()

			Args:
				None
			Returns:
				None

			Interrupt current planning (or execution).
			)pbdoc")
		.def("publish", [](Task& self, const SolutionBasePtr& solution) {
			self.introspection().publishSolution(*solution);
		}, R"pbdoc(
			publish(solution)

			Args:
				solution (Solution): Publish solution
			Returns:
				None

			Publish a given solution to the solution ros topic.
		)pbdoc")
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
			ROS_INFO("Executed successfully.");
		}, R"pbdoc(
			execute(solution)

			Args:
				solution (Solution): Solution, which should be executed.
			Returns:
				None

			Execute Solution.
		)pbdoc");
	// clang-format on
}
}  // namespace python
}  // namespace moveit
