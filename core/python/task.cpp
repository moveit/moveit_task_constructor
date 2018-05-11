#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/task.h>
#include <boost/python.hpp>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

const std::string& Task_name(const Task& self) { return self.name(); }
void Task_add(Task& self, std::auto_ptr<Stage> stage) {
	return self.add(std::unique_ptr<Stage>{stage.release()});
}

} // anonymous namespace

void export_task()
{
	PropertyMap& (Stage::*Stage_getPropertyMap)() = &Stage::properties;  // resolve method ambiguity
	bp::class_<Stage, boost::noncopyable>("Stage", bp::no_init)
	      // expose name as writeable property, returned string reference will be copied
	      .add_property("name",
	                    bp::make_function(&Stage::name, bp::return_value_policy<bp::copy_const_reference>()),
	                    &Stage::setName)
	      // read-only access to properties, reference returned directly as pointer
	      .add_property("properties", bp::make_function(Stage_getPropertyMap, bp::return_internal_reference<>()))
	;

	PropertyMap& (Task::*Task_getPropertyMap)() = &Task::properties;
	bp::class_<Task, boost::noncopyable>("Task", bp::init<bp::optional<const std::string&>>())
	      .add_property("name", bp::make_function(Task_name, bp::return_value_policy<bp::copy_const_reference>()))
	      .add_property("properties", bp::make_function(Task_getPropertyMap, bp::return_internal_reference<>()))

	      .def("loadRobotModel", &Task::loadRobotModel)
	      .def("enableIntrospection", &Task::enableIntrospection)
	      .def("clear", &Task::clear)
	      .def("reset", &Task::reset)
	      .def("init", &Task::init)
	      .def("plan", &Task::plan)
	      .def("add", &Task_add)
	;
}

} }
