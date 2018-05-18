#include <boost/python.hpp>

#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/task.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

void export_properties();
void export_solvers();

namespace {

void ContainerBase_insert(ContainerBase& self, std::auto_ptr<Stage> stage, int before = -1) {
	self.insert(std::unique_ptr<Stage>{stage.release()}, before);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(ContainerBase_insert_overloads, ContainerBase_insert, 2, 3)


// TODO task constructor optional/default values
Task* Task_init(const std::string& id, std::auto_ptr<ContainerBase> container) {
	return new Task(id, std::unique_ptr<ContainerBase>{container.release()});
}

void Task_add(Task& self, std::auto_ptr<Stage> stage) {
	return self.add(std::unique_ptr<Stage>{stage.release()});
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(Task_enableIntrospection_overloads, Task::enableIntrospection, 0, 1)

}

void export_core()
{
	PropertyMap& (Stage::*Stage_getPropertyMap)() = &Stage::properties;  // resolve method ambiguity
	properties::class_<Stage, std::auto_ptr<Stage>, boost::noncopyable>
	      ("Stage", bp::no_init)
	      // expose name as writeable property, returned string reference will be copied
	      .property<double>("timeout")
	      .add_property("name",
	                    bp::make_function(&Stage::name, bp::return_value_policy<bp::copy_const_reference>()),
	                    &Stage::setName)
	      // read-only access to properties, reference returned directly as pointer
	      .add_property("properties", bp::make_function(Stage_getPropertyMap, bp::return_internal_reference<>()))
	      .def("reset", &Stage::reset)
	      .def("init", &Stage::init)
	      ;


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
	      .add_property("properties", bp::make_function(Task_getPropertyMap, bp::return_internal_reference<>()))

	      .def("__init__", bp::make_constructor(&Task_init))
	      .def("loadRobotModel", &Task::loadRobotModel)
	      .def("enableIntrospection", &Task::enableIntrospection, Task_enableIntrospection_overloads())
	      .def("clear", &Task::clear)
	      .def("reset", &Task::reset)
	      .def("init", &Task::init)
	      .def("plan", &Task::plan)
	      .def("add", &Task_add)
	      ;
}

} }
