#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>

#include <geometry_msgs/PoseStamped.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

bp::object PropertyMap_get(const PropertyMap& self, const std::string& name) {
	const Property& prop = self.property(name);
	const boost::any& value = prop.value();
	const std::string& type_name = value.type().name();

	/// type-casting for selected primitive types
	if (type_name == typeid(bool).name())
		return bp::object(boost::any_cast<bool>(value));
	else if (type_name == typeid(int).name())
		return bp::object(boost::any_cast<int>(value));
	else if (type_name == typeid(unsigned int).name())
		return bp::object(boost::any_cast<unsigned int>(value));
	else if (type_name == typeid(long).name())
		return bp::object(boost::any_cast<long>(value));
	else if (type_name == typeid(float).name())
		return bp::object(boost::any_cast<float>(value));
	else if (type_name == typeid(double).name())
		return bp::object(boost::any_cast<double>(value));

	/// type-casting for selected ROS msg types
	else if (type_name == typeid(geometry_msgs::Pose).name())
		return toPython("geometry_msgs/Pose", boost::any_cast<geometry_msgs::Pose>(value));

	throw std::runtime_error("No conversion for: " + type_name);
}

void PropertyMap_set(PropertyMap& self, const std::string& name, const bp::object& value) {
	PyObject *o = value.ptr();
	if (PyBool_Check(o))
		self.set(name, o == Py_True);
	else if (PyInt_Check(o))
		self.set(name, PyInt_AS_LONG(o));
	else if (PyFloat_Check(o))
		self.set(name, PyFloat_AS_DOUBLE(o));
	else if (PyString_Check(o))
		self.set(name, std::string(PyString_AS_STRING(o)));

	else {
		std::string python_type_name = bp::extract<std::string>(value.attr("__class__").attr("__module__"));
		std::string ros_msg_name = rosMsgName(python_type_name);
		if (ros_msg_name == "geometry_msgs/Pose")
			self.set(name, fromPython<geometry_msgs::Pose>(value));
		else
			throw std::runtime_error("No conversion for: " + python_type_name);
	}
}

} // anonymous namespace

void export_properties()
{
	bp::class_<PropertyMap, boost::noncopyable>("PropertyMap")
	      .def("__getitem__", &PropertyMap_get)
	      .def("__setitem__", &PropertyMap_set)
	;
}

} }
