#include <boost/python.hpp>

#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/Solution.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

// utility function to extract index from python object
// also handles negative indexes referencing from the end
size_t convert_index(long size, PyObject* i_) {
	bp::extract<long> i(i_);
	if (i.check()) {
		long index = i();
		if (index < 0)
			index += size;
		if (index >= long(size) || index < 0) {
			PyErr_SetString(PyExc_IndexError, "Index out of range");
			bp::throw_error_already_set();
		}
		return index;
	}

	PyErr_SetString(PyExc_TypeError, "Invalid index type");
	bp::throw_error_already_set();
	return size_t();
}

// implement operator[](index)
template<typename T>
typename T::value_type get_item(const T& container, PyObject* i) {
	auto it = container.begin();
	std::advance(it, convert_index(container.size(), i));
	return *it;
}

// provide implicit type conversion from std::shared_ptr<const T> to std::shared_ptr<T>
template<typename T>
struct const_castable {
	typedef std::shared_ptr<const T> Source;
	typedef std::shared_ptr<T> Target;

	static PyObject* convert(const Source& x) {
		return bp::incref(bp::object(std::const_pointer_cast<T>(x)).ptr());
	}

	// register type conversion
	const_castable() {
		bp::to_python_converter<Source, const_castable<T>>();
	}
};


void InitStageException_translator(InitStageException const& e) {
	std::stringstream message;
	message << e;
	PyErr_SetString(PyExc_UserWarning, message.str().c_str());
}


moveit_task_constructor_msgs::Solution SolutionBase_toMsg(SolutionBasePtr s) {
	moveit_task_constructor_msgs::Solution msg;
	s->fillMessage(msg);
	return msg;
}


bp::list Stage_getForwardedProperties(Stage& self) {
	bp::list l;
	for (const std::string& value : self.forwardedProperties())
		l.append(value);
	return l;
}

void Stage_setForwardedProperties(Stage& self, const bp::list& names) {
	boost::python::stl_input_iterator<std::string> begin(names), end;
	self.setForwardedProperties(std::set<std::string>(begin, end));
}


void ContainerBase_insert(ContainerBase& self, std::auto_ptr<Stage> stage, int before = -1) {
	self.insert(std::unique_ptr<Stage>{stage.release()}, before);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ContainerBase_insert_overloads, ContainerBase_insert, 2, 3)


Task* Task_init(const std::string& id, std::auto_ptr<ContainerBase> container) {
	return new Task(id, std::unique_ptr<ContainerBase>{container.release()});
}

void Task_add(Task& self, std::auto_ptr<Stage> stage) {
	return self.add(std::unique_ptr<Stage>{stage.release()});
}

void Task_publish(Task& self, SolutionBasePtr &solution) {
	self.introspection().publishSolution(*solution);
}

void Task_execute(Task& self, SolutionBasePtr &solution) {
	moveit::planning_interface::PlanningSceneInterface psi;
	moveit::planning_interface::MoveGroupInterface mgi(solution->start()->scene()->getRobotModel()->getJointModelGroupNames()[0]);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	moveit_task_constructor_msgs::Solution serialized;
	solution->fillMessage(serialized);

	for(const moveit_task_constructor_msgs::SubTrajectory& traj : serialized.sub_trajectory) {
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
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Task_plan_overloads, Task::plan, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Task_enableIntrospection_overloads, Task::enableIntrospection, 0, 1)

}

void export_core()
{
	bp::register_exception_translator<InitStageException>(InitStageException_translator);
	RosMsgConverter<moveit_task_constructor_msgs::Solution>();

	bp::scope().attr("PARENT") = static_cast<unsigned int>(Stage::PARENT);
	bp::scope().attr("INTERFACE") = static_cast<unsigned int>(Stage::INTERFACE);

	bp::class_<SolutionBase, SolutionBasePtr, boost::noncopyable>("Solution", bp::no_init)
	      .add_property("cost", &SolutionBase::cost)
	      .add_property("comment", bp::make_function(&SolutionBase::comment, bp::return_value_policy<bp::copy_const_reference>()))
	      .def("toMsg", &SolutionBase_toMsg)
	      ;
	const_castable<SolutionBase>();


	typedef ordered<SolutionBaseConstPtr> Solutions;
	bp::class_<Solutions, boost::noncopyable>("Solutions", bp::no_init)
	      .def("__len__", &Solutions::size)
	      .def("__getitem__", &get_item<Solutions>)
	      .def("__iter__", bp::iterator<Solutions>())
	      ;


	PropertyMap& (Stage::*Stage_getPropertyMap)() = &Stage::properties;  // resolve method ambiguity
	properties::class_<Stage, std::auto_ptr<Stage>, boost::noncopyable>
	      ("Stage", bp::no_init)
	      .property<double>("timeout")
	      .property<std::string>("marker_ns")
	      // expose name as writeable property, returned string reference will be copied
	      .add_property("name",
	                    bp::make_function(&Stage::name, bp::return_value_policy<bp::copy_const_reference>()),
	                    &Stage::setName)
	      // read-only access to properties + solutions, reference returned directly as pointer
	      .add_property("properties", bp::make_function(Stage_getPropertyMap, bp::return_internal_reference<>()))
	      .add_property("forwarded_properties", &Stage_getForwardedProperties, &Stage_setForwardedProperties)
	      .add_property("solutions", bp::make_function(&Stage::solutions, bp::return_internal_reference<>()))
	      .add_property("failures", bp::make_function(&Stage::failures, bp::return_internal_reference<>()))
	      .def("reset", &Stage::reset)
	      .def("init", &Stage::init)
	      ;


	bp::enum_<PropagatingEitherWay::Direction>("PropagationDirection")
	      .value("AUTO", PropagatingEitherWay::AUTO)
	      .value("FORWARD", PropagatingEitherWay::FORWARD)
	      .value("BACKWARD", PropagatingEitherWay::BACKWARD)
	      .value("BOTHWAY", PropagatingEitherWay::BOTHWAY)
	      ;

	bp::class_<PropagatingEitherWay, std::auto_ptr<PropagatingEitherWay>, bp::bases<Stage>, boost::noncopyable>
	      ("PropagatingEitherWay", bp::no_init)
	      .def("restrictDirection", &PropagatingEitherWay::restrictDirection)
	      ;
	bp::implicitly_convertible<std::auto_ptr<PropagatingEitherWay>, std::auto_ptr<Stage>>();


	bp::class_<MonitoringGenerator, std::auto_ptr<MonitoringGenerator>, bp::bases<Stage>, boost::noncopyable>
	      ("MonitoringGenerator", bp::no_init)
	      .def("setMonitoredStage", &MonitoringGenerator::setMonitoredStage)
	      ;
	bp::implicitly_convertible<std::auto_ptr<MonitoringGenerator>, std::auto_ptr<Stage>>();


	bp::class_<ContainerBase, std::auto_ptr<ContainerBase>, bp::bases<Stage>, boost::noncopyable>
	      ("ContainerBase", bp::no_init)
	      .def("insert", &ContainerBase_insert, ContainerBase_insert_overloads())
	      .def("clear", &ContainerBase::clear)
	      ;
	bp::implicitly_convertible<std::auto_ptr<ContainerBase>, std::auto_ptr<Stage>>();


	bp::class_<SerialContainer, std::auto_ptr<SerialContainer>, bp::bases<ContainerBase>, boost::noncopyable>
	      ("SerialContainer", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<SerialContainer>, std::auto_ptr<ContainerBase>>();


	bp::class_<ParallelContainerBase, std::auto_ptr<ParallelContainerBase>, bp::bases<ContainerBase>, boost::noncopyable>
	      ("ParallelContainerBase", bp::no_init)
	      ;
	bp::implicitly_convertible<std::auto_ptr<ParallelContainerBase>, std::auto_ptr<ContainerBase>>();


	bp::class_<Alternatives, bp::bases<ParallelContainerBase>, boost::noncopyable>
	      ("Alternatives", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<Alternatives>, std::auto_ptr<ParallelContainerBase>>();


	bp::class_<Fallbacks, bp::bases<ParallelContainerBase>, boost::noncopyable>
	      ("Fallbacks", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<Fallbacks>, std::auto_ptr<ParallelContainerBase>>();


	bp::class_<Merger, std::auto_ptr<Merger>, bp::bases<ParallelContainerBase>, boost::noncopyable>
	      ("Merger", bp::init<bp::optional<const std::string&>>())
	      ;
	bp::implicitly_convertible<std::auto_ptr<Merger>, std::auto_ptr<ParallelContainerBase>>();


	bp::class_<WrapperBase, std::auto_ptr<WrapperBase>, bp::bases<ParallelContainerBase>, boost::noncopyable>
	      ("WrapperBase", bp::no_init)
	      ;
	bp::implicitly_convertible<std::auto_ptr<WrapperBase>, std::auto_ptr<ParallelContainerBase>>();


	PropertyMap& (Task::*Task_getPropertyMap)() = &Task::properties;
	bp::class_<Task, boost::noncopyable>
	      ("Task", bp::no_init)
	      .add_property("id", &Task::id)
	      // read-only access to properties + solutions, reference returned directly as pointer
	      .add_property("properties", bp::make_function(Task_getPropertyMap, bp::return_internal_reference<>()))
	      .add_property("solutions", bp::make_function(&Task::solutions, bp::return_internal_reference<>()))
	      .add_property("failures", bp::make_function(&Task::failures, bp::return_internal_reference<>()))

	      .def("__init__", bp::make_constructor(&Task_init))
	      .def(bp::init<bp::optional<const std::string&>>())
	      .def("loadRobotModel", &Task::loadRobotModel)
	      .def("enableIntrospection", &Task::enableIntrospection, Task_enableIntrospection_overloads())
	      .def("clear", &Task::clear)
	      .def("add", &Task_add)
	      .def("reset", &Task::reset)
	      .def("init", &Task::init)
	      .def("plan", &Task::plan, Task_plan_overloads())
	      .def("preempt", &Task::preempt)
	      .def("publish", &Task_publish)
	      .def("execute", &Task_execute)
	      ;
}

} }
