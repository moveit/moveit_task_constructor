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
	py::class_<SolutionBase, SolutionBasePtr>(m, "Solution")
		.def_property("cost", &SolutionBase::cost, &SolutionBase::setCost)
		.def_property("comment", &SolutionBase::comment, &SolutionBase::setComment)
		.def("toMsg", [](const SolutionBasePtr& s) {
			moveit_task_constructor_msgs::Solution msg;
			s->fillMessage(msg);
			return msg;
		})
		;
	py::class_<SubTrajectory, SubTrajectoryPtr, SolutionBase>(m, "SubTrajectory")
		.def(py::init<>())
		;

	typedef ordered<SolutionBaseConstPtr> Solutions;
	py::class_<Solutions>(m, "Solutions")
		.def("__len__", &Solutions::size)
		.def("__getitem__", &get_item<Solutions>)
		.def("__iter__", [](Solutions& self) { return py::make_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())
		;

	py::class_<InterfaceState>(m, "InterfaceState")
		;

	auto stage = properties::class_<Stage, PyStage<>>(m, "Stage")
		.property<double>("timeout")
		.property<std::string>("marker_ns")
		.def_property("forwarded_properties", getForwardedProperties, setForwardedProperties)
		// expose name as writeable property
		.def_property("name", &Stage::name, &Stage::setName)
		// read-only access to properties + solutions
		.def_property_readonly<const PropertyMap& (Stage::*)() const>("properties", &Stage::properties, py::return_value_policy::reference_internal)
		.def_property_readonly("solutions", &Stage::solutions, py::return_value_policy::reference_internal)
		.def_property_readonly("failures", &Stage::failures, py::return_value_policy::reference_internal)
		.def("reset", &Stage::reset)
		.def("init", &Stage::init);

	py::enum_<Stage::PropertyInitializerSource>(stage, "PropertyInitializerSource")
		.value("PARENT", Stage::PARENT)
		.value("INTERFACE", Stage::INTERFACE)
		;


	auto either_way = py::class_<PropagatingEitherWay, Stage, PyPropagatingEitherWay<>>(m, "PropagatingEitherWay")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingEitherWay"))
		.def("restrictDirection", &PropagatingEitherWay::restrictDirection)
		.def("computeForward", &PropagatingEitherWay::computeForward)
		.def("computeBackward", &PropagatingEitherWay::computeBackward)
		//.def("sendForward", &PropagatingEitherWay::sendForward)
		//.def("sendBackward", &PropagatingEitherWay::sendBackward)
		;

	py::enum_<PropagatingEitherWay::Direction>(either_way, "Direction")
		.value("AUTO", PropagatingEitherWay::AUTO)
		.value("FORWARD", PropagatingEitherWay::FORWARD)
		.value("BACKWARD", PropagatingEitherWay::BACKWARD);

	py::class_<PropagatingForward, Stage, PyPropagatingEitherWay<PropagatingForward>>(m, "PropagatingForward")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingForward"))
		;
	py::class_<PropagatingBackward, Stage, PyPropagatingEitherWay<PropagatingBackward>>(m, "PropagatingBackward")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("PropagatingBackward"))
		;

	properties::class_<Generator, Stage, PyGenerator<>>(m, "Generator")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("generator"))
		.def("canCompute", &Generator::canCompute)
		.def("compute", &Generator::compute)
		;

	properties::class_<MonitoringGenerator, Generator, PyMonitoringGenerator<>>(m, "MonitoringGenerator")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("generator"))
		.def("setMonitoredStage", &MonitoringGenerator::setMonitoredStage)
		.def("_onNewSolution", &PubMonitoringGenerator::onNewSolution)
		;

	py::class_<ContainerBase, Stage>(m, "ContainerBase")
		.def("add", &ContainerBase::add)
		.def("insert", &ContainerBase::insert, py::arg("stage"), py::arg("before") = -1)
		.def("remove", static_cast<Stage::pointer (ContainerBase::*)(int)>(&ContainerBase::remove))
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

	py::class_<SerialContainer, ContainerBase>(m, "SerialContainer")
	    .def(py::init<const std::string&>(), py::arg("name") = std::string("serial container"));

	py::class_<ParallelContainerBase, ContainerBase>(m, "ParallelContainerBase");

	py::class_<Alternatives, ParallelContainerBase>(m, "Alternatives")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("alternatives"));

	py::class_<Fallbacks, ParallelContainerBase>(m, "Fallbacks")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("fallbacks"));

	py::class_<Merger, ParallelContainerBase>(m, "Merger")
		.def(py::init<const std::string&>(), py::arg("name") = std::string("merger"));

	py::class_<WrapperBase, ParallelContainerBase>(m, "WrapperBase");

	py::class_<Task>(m, "Task")
		.def(py::init<const std::string&, bool>(), py::arg("ns") = std::string(), py::arg("introspection") = true)
		.def(py::init<const std::string&, bool, ContainerBase::pointer&&>(),
		     py::arg("ns") = std::string(), py::arg("introspection") = true, py::arg("container"))
		// read-only access to properties + solutions
		.def_property_readonly<const PropertyMap& (Task::*)() const>("properties", &Task::properties, py::return_value_policy::reference_internal)
		.def_property_readonly("solutions", &Task::solutions, py::return_value_policy::reference_internal)
		.def_property_readonly("failures", &Task::failures, py::return_value_policy::reference_internal)
		.def_property("name", &Task::name, &Task::setName)
		.def("loadRobotModel", &Task::loadRobotModel)
		.def("enableIntrospection", &Task::enableIntrospection, py::arg("enabled") = true)
		.def("clear", &Task::clear)
		.def("add", &Task::add)
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
		.def("reset", &Task::reset)
		.def("init", &Task::init)
		.def("plan", &Task::plan, py::arg("max_solutions") = 0)
		.def("preempt", &Task::preempt)
		.def("publish", [](Task& self, const SolutionBasePtr& solution) {
			self.introspection().publishSolution(*solution);
		})
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
		});
	// clang-format on
}
}  // namespace python
}  // namespace moveit
