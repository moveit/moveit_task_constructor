/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Bielefeld University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <moveit/python/task_constructor/properties.h>
#include <boost/core/demangle.hpp>

namespace py = pybind11;
using namespace py::literals;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {
namespace {

/** In order to assign new property values in Python, we need to convert the Python object
 *  to a boost::any instance of the correct type. As the C++ type cannot be inferred from
 *  the Python type, we can support this assignment only for a few basic types (see fromPython())
 *  as well as ROS message types. For other types, a generic assignment via
 *  stage.properties["property"] = value
 *  is not possible. Instead, use the .property<Type> declaration on the stage to allow for
 *  direct assignment like this: stage.property = value
 **/
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
	using RosMsgTypeNameMap = std::map<std::string, RegistryMap::iterator>;
	RosMsgTypeNameMap msg_names_;

public:
	PropertyConverterRegistry();

	inline bool insert(const std::type_index& type_index, const std::string& ros_msg_name,
	                   PropertyConverterBase::to_python_converter_function to,
	                   PropertyConverterBase::from_python_converter_function from);

	static py::object toPython(const boost::any& value);

	static boost::any fromPython(const py::object& bpo);
};
static PropertyConverterRegistry REGISTRY_SINGLETON;

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

	auto it = REGISTRY_SINGLETON.types_.find(value.type());
	if (it == REGISTRY_SINGLETON.types_.end()) {
		std::string name = boost::core::demangle(value.type().name());
		throw py::type_error("No Python -> C++ conversion for: " + name);
	}

	return it->second.to_(value);
}

std::string rosMsgName(PyObject* object) {
	py::object o = py::reinterpret_borrow<py::object>(object);
	auto cls = o.attr("__class__");
	auto name = cls.attr("__name__").cast<std::string>();
	auto module = cls.attr("__module__").cast<std::string>();
	auto pos = module.find(".msg");
	if (pos == std::string::npos)
		// object is not a ROS message type, return it's class name instead
		return module + "." + name;
	else
		return module.substr(0, pos) + "/msg/" + name;
}

boost::any PropertyConverterRegistry::fromPython(const py::object& po) {
	PyObject* o = po.ptr();

	if (PyBool_Check(o))
		return (o == Py_True);
	if (PyLong_Check(o))
		return PyLong_AS_LONG(o);
	if (PyFloat_Check(o))
		return PyFloat_AS_DOUBLE(o);
	if (PyUnicode_Check(o))
		return py::cast<std::string>(o);

	const std::string& ros_msg_name = rosMsgName(o);
	auto it = REGISTRY_SINGLETON.msg_names_.find(ros_msg_name);
	if (it == REGISTRY_SINGLETON.msg_names_.end())
		throw py::type_error("No C++ conversion available for (property) type: " + ros_msg_name);

	return it->second->second.from_(po);
}

}  // end anonymous namespace

bool PropertyConverterBase::insert(const std::type_index& type_index, const std::string& ros_msg_name,
                                   moveit::python::PropertyConverterBase::to_python_converter_function to,
                                   moveit::python::PropertyConverterBase::from_python_converter_function from) {
	return REGISTRY_SINGLETON.insert(type_index, ros_msg_name, to, from);
}

void export_properties(py::module& m) {
	// clang-format off
	py::classh<Property>(m, "Property", "Holds an arbitrarily typed value and a default value")
		.def(py::init<>())
		.def("setValue", [](Property& self, const py::object& value)
		     { self.setValue(PropertyConverterRegistry::fromPython(value)); },
		     "Set current and default value.", "value"_a)
		.def("setCurrentValue", [](Property& self, const py::object& value)
	        { self.setCurrentValue(PropertyConverterRegistry::fromPython(value)); },
		     "Set the current value only, w/o touching the default.", "value"_a)
		.def("value", [](const Property& self)
		     { return PropertyConverterRegistry::toPython(self.value()); }, "Retrieve the stored value.")
		.def("defaultValue", [](const Property& self)
		     { return PropertyConverterRegistry::toPython(self.defaultValue()); },
		     "Retrieve the default value.")
		.def("reset", &Property::reset, "Reset the value to the stored default.")
		.def("defined", &Property::defined, "Was a (non-default) value stored?")
		.def("description", &Property::description, "Retrive the property description string")
		.def("setDescription", &Property::setDescription,
		     "Set the property's description", "desc"_a);

	py::classh<PropertyMap>(m, "PropertyMap", "Dictionary of named :doc:`properties <pymoveit_mtc.core.Property>`")
		.def(py::init<>())
		.def("__bool__", [](const PropertyMap& self) { return self.begin() == self.end();})
		.def("__iter__", [](PropertyMap& self) { return py::make_key_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>())  // Essential: keep list alive while iterator exists
		.def("items", [](const PropertyMap& self) { return py::make_iterator(self.begin(), self.end()); },
		     py::keep_alive<0, 1>(), "Retrieve an iterator over the items of the dictionary.")
		.def("__contains__", [](const PropertyMap& self, const std::string &key) { return self.hasProperty(key); })
		.def("property", [](PropertyMap& self, const std::string& key)
		     { return self.property(key); }, py::return_value_policy::reference_internal, R"(
		    Retrieve the property instance for the given key.
		    This is in contrast to ``map[key]``, which returns ``map.property(key).value()``.)",
		    "key"_a)
		.def("__getitem__", [](const PropertyMap& self, const std::string& key)
		     { return PropertyConverterRegistry::toPython(self.get(key)); })
		.def("__setitem__", [](PropertyMap& self, const std::string& key, const py::object& value)
		     { self.set(key, PropertyConverterRegistry::fromPython(value)); })
		.def("reset", &PropertyMap::reset, "Reset all properties to their default values")
		.def("update", [](PropertyMap& self, const py::dict& values) {
				for (auto it = values.begin(), end = values.end(); it != end; ++it) {
					self.set(it->first.cast<std::string>(),
					         PropertyConverterRegistry::fromPython(py::reinterpret_borrow<py::object>(it->second)));
				}
			}, "Update property values from another dictionary", "values"_a)
		.def("configureInitFrom", [](PropertyMap& self, Property::SourceFlags sources, const py::list& names) {
				std::set<std::string> s;
				for (auto& item : names)
					s.insert(item.cast<std::string>());
				self.configureInitFrom(sources, s);
			}, "Configure initialization of listed (or all) properties from given source(s).",
			"sources"_a, "names"_a = py::list())
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const std::string& name) {
				self.exposeTo(other, name, name);
			}, "Declare ``named`` property in ``other`` PropertyMap - using same name.",
			"other"_a, "name"_a)
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const std::string& name, const std::string& other_name) {
				self.exposeTo(other, name, other_name);
			}, "Declare ``named`` property in ``other`` PropertyMap - using ``other_name``.",
			"other"_a, "name"_a, "other_name"_a)
		.def("exposeTo", [](PropertyMap& self, PropertyMap& other, const py::list& names) {
				std::set<std::string> s;
				for (auto& item : names)
					s.insert(item.cast<std::string>());
				self.exposeTo(other, s);
			}, "Declare `all` ``named`` properties in ``other`` PropertyMap - using the same names.",
			"other"_a, "names"_a)
		;
	// clang-format on
}
}  // namespace python
}  // namespace moveit
