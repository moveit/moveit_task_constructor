#include <moveit/task_constructor/task.h>
#include <boost/python.hpp>

namespace bp = boost::python;
namespace mtc = moveit::task_constructor;

namespace moveit {
namespace python {

void export_task()
{
	bp::class_<mtc::Task, boost::noncopyable>("Task", bp::init<bp::optional<const std::string&>>())
	      .def("loadRobotModel", &mtc::Task::loadRobotModel)
	      .def("enableIntrospection", &mtc::Task::enableIntrospection)
	      .def("clear", &mtc::Task::clear)
	      .def("reset", &mtc::Task::reset)
	      .def("init", &mtc::Task::init)
	      .def("plan", &mtc::Task::plan)
	;
}

} }
