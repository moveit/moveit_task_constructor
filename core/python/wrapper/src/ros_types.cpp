#include <moveit/python/python_tools/ros_types.h>

namespace py = pybind11;
namespace moveit {
namespace python {

py::object createMessage(const std::string& ros_msg_name) {
	// find delimiting '/' in ros msg name
	std::size_t pos = ros_msg_name.find('/');
	// import module
	py::module m = py::module::import((ros_msg_name.substr(0, pos) + ".msg").c_str());
	// retrieve type instance
	py::object cls = m.attr(ros_msg_name.substr(pos + 1).c_str());
	// create message instance
	return cls();
}

bool convertible(const pybind11::handle& h, const char* ros_msg_name) {
	try {
		PyObject* o = h.attr("_type").ptr();
		return PyString_Check(o) && strcmp(PyString_AS_STRING(o), ros_msg_name) == 0;
	} catch (const std::exception& e) {
		return false;
	}
}
}  // namespace python
}  // namespace moveit
