#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>

#include <geometry_msgs/PoseStamped.h>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

bp::object property_value_to_python(const boost::any& value) {
	if (value.empty())
		return bp::object();

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

void property_value_from_python(const bp::object& bpo, boost::any& value) {
	PyObject *o = bpo.ptr();
	if (PyBool_Check(o))
		value = (o == Py_True);
	else if (PyInt_Check(o))
		value = PyInt_AS_LONG(o);
	else if (PyFloat_Check(o))
		value = PyFloat_AS_DOUBLE(o);
	else if (PyString_Check(o))
		value = std::string(PyString_AS_STRING(o));

	else {
		std::string python_type_name = bp::extract<std::string>(bpo.attr("__class__").attr("__module__"));
		if (python_type_name == "geometry_msgs.msg._Pose")
			value = geometry_msgs::Pose(boost::python::extract<geometry_msgs::Pose>(bpo));
		else
			throw std::runtime_error("No conversion for: " + python_type_name);
	}
}

struct property_pair_to_python {
	static PyObject* convert(const PropertyMap::iterator::value_type& x) {
		return bp::incref(bp::make_tuple<std::string, bp::object>(x.first, property_value_to_python(x.second.value())).ptr());
	}
};

class ConverterInit {
public:
	ConverterInit() {
		using namespace boost::python;
		to_python_converter<PropertyMap::iterator::value_type, property_pair_to_python>();
	}
};
static ConverterInit init;


bp::object PropertyMap_get(const PropertyMap& self, const std::string& name) {
	return property_value_to_python(self.get(name));
}

void PropertyMap_set(PropertyMap& self, const std::string& name, const bp::object& value) {
	boost::any v;
	property_value_from_python(value, v);
	self.set(name, v);
}

void PropertyMap_update(PropertyMap& self, bp::dict values) {
	for (boost::python::stl_input_iterator<boost::python::tuple> it(values.iteritems()), end; it != end; ++it) {
		const std::string& key = boost::python::extract<std::string>((*it)[0]);
		const bp::object& value = boost::python::extract<bp::object>((*it)[1]);
		PropertyMap_set(self, key, value);
	}
}

void PropertyMap_configureInitFrom(PropertyMap& self, Property::SourceFlags sources, bp::list values = bp::list()) {
	boost::python::stl_input_iterator<std::string> begin(values), end;
	self.configureInitFrom(sources, std::set<std::string>(begin, end));
}
BOOST_PYTHON_FUNCTION_OVERLOADS(PropertyMap_configureInitFrom_overloads,
                                PropertyMap_configureInitFrom, 2, 3);

void PropertyMap_exposeTo_1(PropertyMap& self, PropertyMap& other, const std::string& name) {
	self.exposeTo(other, name, name);
}
void PropertyMap_exposeTo_2(PropertyMap& self, PropertyMap& other, const std::string& name, const std::string& other_name) {
	self.exposeTo(other, name, other_name);
}
void PropertyMap_exposeTo_l(PropertyMap& self, PropertyMap& other, bp::list names) {
	boost::python::stl_input_iterator<std::string> begin(names), end;
	self.exposeTo(other, std::set<std::string>(begin, end));
}

} // anonymous namespace

void export_properties()
{
#if 0 // TODO
	bp::class_<Property, boost::noncopyable>("Property", bp::no_init)
	      .def("setValue", &Property::setValue)
	      .def("setCurrentValue", &Property::setCurrentValue)
	      .def("value", &Property::value)
	      .def("defaultValue", &Property::defaultValue)
	      .def("reset", &Property::reset)
	      .def("defined", &Property::defined)
	      .add_property("description", &Property::description, &Property::setDescription)
	      ;
#endif

	bp::class_<PropertyMap, boost::noncopyable>("PropertyMap")
	      .def("__getitem__", &PropertyMap_get)
	      .def("__setitem__", &PropertyMap_set)
	      .def("reset", &PropertyMap::reset, "reset all properties to their defaults")
	      .def("update", &PropertyMap_update)
	      .def("__iter__", bp::iterator<PropertyMap>())
	      .def("configureInitFrom", &PropertyMap_configureInitFrom,
	           PropertyMap_configureInitFrom_overloads())
	      .def("exposeTo", &PropertyMap_exposeTo_1)
	      .def("exposeTo", &PropertyMap_exposeTo_2)
	      .def("exposeTo", &PropertyMap_exposeTo_l)
	      ;
}

} }
