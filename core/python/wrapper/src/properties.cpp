#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>

#include <geometry_msgs/PoseStamped.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

// Used to pass an iterator to its own __iter__ function
inline boost::any identity(const boost::any& self) { return self; }


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
	else if (type_name == typeid(std::string).name())
		return bp::object(boost::any_cast<std::string>(value));

	/// type-casting for selected ROS msg types
	else if (type_name == typeid(geometry_msgs::Pose).name())
		return bp::object(boost::any_cast<geometry_msgs::Pose>(value));

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
		if (python_type_name == "geometry_msgs.msg._Pose")
			self.set<geometry_msgs::Pose>(name, boost::python::extract<geometry_msgs::Pose>(value));
		else
			throw std::runtime_error("No conversion for: " + python_type_name);
	}
}

void PropertyMap_update(PropertyMap& self, bp::dict values) {
	for (boost::python::stl_input_iterator<boost::python::tuple> it(values.iteritems()), end; it != end; ++it) {
		const std::string& key = boost::python::extract<std::string>((*it)[0]);
		const bp::object& value = boost::python::extract<bp::object>((*it)[1]);
		PropertyMap_set(self, key, value);
	}
}

class PropertyMapIterator {
	PropertyMap& pm;
	std::map<std::string, Property>::iterator iterator;
	std::map<std::string, Property>::iterator end;

public:
	PropertyMapIterator(PropertyMap& pm_)  : pm(pm_) {
		iterator = pm.begin();
		// TODO Robert: get end of iterator here or for each call of next function?
		end = pm.end();
	}

	bp::tuple next() {
		if (iterator == end) {
			PyErr_SetString(PyExc_StopIteration, "Iterator exhausted.");
			bp::throw_error_already_set();
		} else {
			const std::string& name = (*iterator).first;
			iterator++;
			return bp::make_tuple<std::string, bp::object>(name, PropertyMap_get(pm, name));
		}
	}
};

PropertyMapIterator* PropertyMap_getIterator(PropertyMap& self) {
	return new PropertyMapIterator(self);
}

} // anonymous namespace

void export_properties()
{
	bp::class_<PropertyMapIterator, boost::noncopyable>
	      ("PropertyMapIterator", bp::init<PropertyMap&>())
	      .def("__iter__", &identity)
	      .def("next", &PropertyMapIterator::next)
	;

	bp::class_<PropertyMap, boost::noncopyable>
	      ("PropertyMap")
	      .def("__getitem__", &PropertyMap_get)
	      .def("__setitem__", &PropertyMap_set)
	      .def("reset", &PropertyMap::reset, "reset all properties to their defaults")
	      .def("__iter__", &PropertyMap_getIterator, bp::return_internal_reference<1>())
	      .def("update", &PropertyMap_update)
	      ;
}

} }
