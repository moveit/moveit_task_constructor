#include <moveit/python/task_constructor/properties.h>
#include <boost/core/demangle.hpp>

namespace py = pybind11;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {
namespace {

class PropertyConverterRegistry
{
	struct Entry
	{
		PropertyConverterBase::to_python_converter_function to_;
		PropertyConverterBase::from_python_converter_function from_;
	};
	// map from type_index to corresponding converter functions
	typedef std::map<std::type_index, Entry> RegistryMap;
	RegistryMap types_;
	// map from ros-msg-names to entry in types_
	typedef std::map<std::string, RegistryMap::iterator> RosMsgTypeNameMap;
	RosMsgTypeNameMap msg_names_;

public:
	PropertyConverterRegistry();

	inline bool insert(const std::type_index& type_index, const std::string& ros_msg_name,
	                   PropertyConverterBase::to_python_converter_function to,
	                   PropertyConverterBase::from_python_converter_function from);

	static py::object toPython(const boost::any& value);

	static boost::any fromPython(const py::object& bpo);
};
static PropertyConverterRegistry registry_singleton_;

PropertyConverterRegistry::PropertyConverterRegistry() {
	// register property converters
	PropertyConverter<bool>();
	PropertyConverter<int>();
	PropertyConverter<unsigned int>();
	PropertyConverter<long>();
	PropertyConverter<float>();
	PropertyConverter<double>();
	PropertyConverter<std::string>();
	PropertyConverter<std::set<std::string>>();
	PropertyConverter<std::map<std::string, double>>();
}

bool PropertyConverterRegistry::insert(const std::type_index& type_index, const std::string& ros_msg_name,
                                       PropertyConverterBase::to_python_converter_function to,
                                       PropertyConverterBase::from_python_converter_function from) {
	auto it_inserted = types_.insert(std::make_pair(type_index, Entry{ to, from }));
	if (!it_inserted.second)
		return false;

	if (!ros_msg_name.empty())  // is this a ROS msg type?
		msg_names_.insert(std::make_pair(ros_msg_name, it_inserted.first));

	return true;
}

py::object PropertyConverterRegistry::toPython(const boost::any& value) {
	if (value.empty())
		return py::object();

	auto it = registry_singleton_.types_.find(value.type());
	if (it == registry_singleton_.types_.end()) {
		std::string msg("No Python -> C++ conversion for: ");
		msg += boost::core::demangle(value.type().name());
		PyErr_SetString(PyExc_TypeError, msg.c_str());
		throw py::error_already_set();
	}

	return it->second.to_(value);
}

std::string rosMsgName(PyObject* object) {
	py::object o = py::reinterpret_borrow<py::object>(object);
	try {
		return o.attr("_type").cast<std::string>();
	} catch (const py::error_already_set&) {
		// change error to TypeError
		std::string msg = o.attr("__class__").attr("__name__").cast<std::string>();
		msg += " is not a ROS message type";
		PyErr_SetString(PyExc_TypeError, msg.c_str());
		throw py::error_already_set();
	}
}

boost::any PropertyConverterRegistry::fromPython(const py::object& po) {
	PyObject* o = po.ptr();

	if (PyBool_Check(o))
		return (o == Py_True);
	if (PyInt_Check(o))
		return PyInt_AS_LONG(o);
	if (PyFloat_Check(o))
		return PyFloat_AS_DOUBLE(o);
	if (PyString_Check(o))
		return std::string(PyString_AS_STRING(o));

	const std::string& ros_msg_name = rosMsgName(o);
	auto it = registry_singleton_.msg_names_.find(ros_msg_name);
	if (it == registry_singleton_.msg_names_.end()) {
		std::string msg("No Python -> C++ conversion for: ");
		msg += ros_msg_name;
		PyErr_SetString(PyExc_TypeError, msg.c_str());
		throw py::error_already_set();
	}

	return it->second->second.from_(po);
}

}  // end anonymous namespace

bool PropertyConverterBase::insert(const std::type_index& type_index, const std::string& ros_msg_name,
                                   moveit::python::PropertyConverterBase::to_python_converter_function to,
                                   moveit::python::PropertyConverterBase::from_python_converter_function from) {
	return registry_singleton_.insert(type_index, ros_msg_name, to, from);
}

void export_properties(py::module& m) {
	// clang-format off
	py::class_<Property>(m, "Property")
		.def("setValue", [](Property& self, const py::object& value)
		     { self.setValue(PropertyConverterRegistry::fromPython(value)); })
		.def("setCurrentValue", [](Property& self, const py::object& value)
	        { self.setCurrentValue(PropertyConverterRegistry::fromPython(value)); })
		.def("value", [](const Property& self)
		     { return PropertyConverterRegistry::toPython(self.value()); })
		.def("value", [](const Property& self)
		     { return PropertyConverterRegistry::toPython(self.defaultValue()); })
		.def("reset", &Property::reset)
		.def("defined", &Property::defined)
		;

	py::class_<PropertyMap>(m, "PropertyMap")
		.def(py::init<>())
		.def("__bool__", [](const PropertyMap& self) { return self.begin() == self.end(); },
		     "Check whether the map is nonempty")
		.def("__iter__", [](PropertyMap& self) { return py::make_key_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())  // Essential: keep list alive while iterator exists
		.def("items", [](const PropertyMap& self) { return py::make_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())
		.def("__contains__", [](const PropertyMap& self, const std::string &key) { return self.hasProperty(key); })
		.def("property", [](PropertyMap& self, const std::string& key)
		     { return self.property(key); }, py::return_value_policy::reference_internal)
		.def("__getitem__", [](const PropertyMap& self, const std::string& key)
		     { return PropertyConverterRegistry::toPython(self.get(key)); })
		.def("__setitem__", [](PropertyMap& self, const std::string& key, const py::object& value)
		     { self.set(key, PropertyConverterRegistry::fromPython(value)); })
		.def("reset", &PropertyMap::reset, "reset all properties to their defaults")
		.def("update", [](PropertyMap& self, const py::dict& values) {
				for (auto it = values.begin(), end = values.end(); it != end; ++it) {
					self.set(it->first.cast<std::string>(),
					         PropertyConverterRegistry::fromPython(py::reinterpret_borrow<py::object>(it->second)));
				}
			})
		.def("configureInitFrom", [](PropertyMap& self, Property::SourceFlags sources, const py::list& names) {
				std::set<std::string> s;
				for (auto& item : names)
					s.insert(item.cast<std::string>());
				self.configureInitFrom(sources, s);
			}, "configure initialization of listed/all properties from given source",
			py::arg("sources"), py::arg("names") = py::list())
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const std::string& name) {
				self.exposeTo(other, name, name);
			})
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const std::string& name, const std::string& other_name) {
				self.exposeTo(other, name, other_name);
			})
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const py::list& names) {
				std::set<std::string> s;
				for (auto& item : names)
					s.insert(item.cast<std::string>());
				self.exposeTo(other, s);
			})
		;
	// clang-format on
}
}  // namespace python
}  // namespace moveit
