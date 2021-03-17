#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <moveit/python/python_tools/ros_init.h>
#include <ros/init.h>

namespace py = pybind11;
using namespace moveit::python;

PYBIND11_MODULE(pymoveit_python_tools, m) {
	m.doc() = "MoveIt python tools";

	m.def("roscpp_init", &InitProxy::init, "Initialize C++ ROS", py::arg("node_name") = "moveit_python_wrapper",
	      py::arg("remappings") = std::map<std::string, std::string>(), py::arg("options") = 0);
	m.def("roscpp_shutdown", &InitProxy::shutdown, "Shutdown C++ ROS");

	py::enum_<ros::InitOption>(m, "InitOption")
	    .value("AnonymousName", ros::init_options::AnonymousName)
	    .value("NoRosout", ros::init_options::NoRosout);
}
