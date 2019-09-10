#include <moveit/python/python_tools/conversions.h>
#include <moveit/python/task_constructor/properties.h>
#include <boost/core/demangle.hpp>

namespace bp = boost::python;
using namespace moveit::task_constructor;

namespace moveit {
namespace python {

namespace {

// Converter for std::set<T> to/from Python list
template <typename T>
struct SetConverter
{
	SetConverter() {
		bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::set<T>>());
		bp::to_python_converter<std::set<T>, SetConverter<T>>();
	}

	// Determine if obj can be converted into std::set
	static void* convertible(PyObject* obj) { return PyList_Check(obj) ? obj : nullptr; }

	/// Conversion from Python to C++
	static void construct(PyObject* obj, bp::converter::rvalue_from_python_stage1_data* data) {
		bp::object bpo(bp::borrowed(obj));
		bp::stl_input_iterator<T> begin(bpo), end;
		// Obtain a pointer to the memory block that the converter has allocated for the C++ type.
		void* storage = reinterpret_cast<bp::converter::rvalue_from_python_storage<std::set<T>>*>(data)->storage.bytes;
		// Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's
		// convertible variable.
		data->convertible = new (storage) std::set<T>(begin, end);
	}

	/// Conversion from C++ object to Python object
	static PyObject* convert(const std::set<T>& v) {
		bp::list l;
		for (const T& value : v)
			l.append(value);
		return bp::incref(l.ptr());
	}
};

// Converter for std::map<K,V> to/from Python list
template <typename K, typename V>
struct MapConverter
{
	MapConverter() {
		bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::map<K, V>>());
		bp::to_python_converter<std::map<K, V>, MapConverter<K, V>>();
	}

	// Determine if obj can be converted into std::map
	static void* convertible(PyObject* obj) { return PyMapping_Check(obj) ? obj : nullptr; }

	/// Conversion from Python to C++
	static void construct(PyObject* obj, bp::converter::rvalue_from_python_stage1_data* data) {
		// Obtain a pointer to the memory block that the converter has allocated for the C++ type.
		typedef bp::converter::rvalue_from_python_storage<std::map<K, V>> storage_t;
		void* storage = reinterpret_cast<storage_t*>(data)->storage.bytes;
		std::map<K, V>* m = new (storage) std::map<K, V>;
		// populate the map
		bp::object items(bp::borrowed(PyMapping_Items(obj)));
		for (bp::stl_input_iterator<bp::tuple> it(items), end; it != end; ++it) {
			const K& key = bp::extract<K>((*it)[0]);
			const V& value = bp::extract<V>((*it)[1]);
			m->insert(std::make_pair(key, value));
		}
		// Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's
		// convertible variable.
		data->convertible = storage;
	}

	/// Conversion from C++ object to Python object
	static PyObject* convert(const std::map<K, V>& v) {
		bp::dict d;
		for (const auto& pair : v)
			d[pair.first] = pair.second;
		return bp::incref(d.ptr());
	}
};

class PropertyConverterRegistry
{
	struct Entry
	{
		PropertyConverterBase::to_python_converter_function to_;
		PropertyConverterBase::from_python_converter_function from_;
	};
	// map from type_info to corresponding converter functions
	typedef std::map<bp::type_info, Entry> RegistryMap;
	RegistryMap types_;
	// map from ros-msg-names to entry in types_
	typedef std::map<std::string, RegistryMap::iterator> RosMsgTypeNameMap;
	RosMsgTypeNameMap msg_names_;

public:
	PropertyConverterRegistry();

	inline bool insert(const bp::type_info& type_info, const std::string& ros_msg_name,
	                   PropertyConverterBase::to_python_converter_function to,
	                   PropertyConverterBase::from_python_converter_function from);

	static bp::object toPython(const boost::any& value);

	static boost::any fromPython(const bp::object& bpo);
};
static PropertyConverterRegistry registry_singleton_;

PropertyConverterRegistry::PropertyConverterRegistry() {
	// register custom type converters
	SetConverter<std::string>();
	MapConverter<std::string, double>();

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

bool PropertyConverterRegistry::insert(const bp::type_info& type_info, const std::string& ros_msg_name,
                                       PropertyConverterBase::to_python_converter_function to,
                                       PropertyConverterBase::from_python_converter_function from) {
	auto it_inserted = types_.insert(std::make_pair(type_info, Entry{ to, from }));
	if (!it_inserted.second)
		return false;

	if (!ros_msg_name.empty())  // is this a ROS msg type?
		msg_names_.insert(std::make_pair(ros_msg_name, it_inserted.first));

	return true;
}

bp::object PropertyConverterRegistry::toPython(const boost::any& value) {
	if (value.empty())
		return bp::object();

	auto it = registry_singleton_.types_.find(value.type());
	if (it == registry_singleton_.types_.end()) {
		std::string msg("No Python -> C++ conversion for: ");
		msg += boost::core::demangle(value.type().name());
		PyErr_SetString(PyExc_TypeError, msg.c_str());
		bp::throw_error_already_set();
	}

	return it->second.to_(value);
}

boost::any PropertyConverterRegistry::fromPython(const bp::object& bpo) {
	PyObject* o = bpo.ptr();

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
		bp::throw_error_already_set();
	}

	return it->second->second.from_(bpo);
}

}  // end anonymous namespace

bool PropertyConverterBase::insert(const bp::type_info& type_info, const std::string& ros_msg_name,
                                   moveit::python::PropertyConverterBase::to_python_converter_function to,
                                   moveit::python::PropertyConverterBase::from_python_converter_function from) {
	return registry_singleton_.insert(type_info, ros_msg_name, to, from);
}

namespace {

struct property_pair_to_python
{
	static PyObject* convert(const PropertyMap::iterator::value_type& x) {
		return bp::incref(
		    bp::make_tuple<std::string, bp::object>(x.first, PropertyConverterRegistry::toPython(x.second.value()))
		        .ptr());
	}
};

bp::object PropertyMap_get(const PropertyMap& self, const std::string& name) {
	return PropertyConverterRegistry::toPython(self.get(name));
}

void PropertyMap_set(PropertyMap& self, const std::string& name, const bp::object& value) {
	self.set(name, PropertyConverterRegistry::fromPython(value));
}

void PropertyMap_update(PropertyMap& self, bp::dict values) {
	for (bp::stl_input_iterator<bp::tuple> it(values.iteritems()), end; it != end; ++it) {
		const std::string& key = bp::extract<std::string>((*it)[0]);
		const bp::object& value = bp::extract<bp::object>((*it)[1]);
		PropertyMap_set(self, key, value);
	}
}

void PropertyMap_configureInitFrom(PropertyMap& self, Property::SourceFlags sources, bp::list values = bp::list()) {
	bp::stl_input_iterator<std::string> begin(values), end;
	self.configureInitFrom(sources, std::set<std::string>(begin, end));
}
BOOST_PYTHON_FUNCTION_OVERLOADS(PropertyMap_configureInitFrom_overloads, PropertyMap_configureInitFrom, 2, 3);

void PropertyMap_exposeTo_1(PropertyMap& self, PropertyMap& other, const std::string& name) {
	self.exposeTo(other, name, name);
}
void PropertyMap_exposeTo_2(PropertyMap& self, PropertyMap& other, const std::string& name,
                            const std::string& other_name) {
	self.exposeTo(other, name, other_name);
}
void PropertyMap_exposeTo_l(PropertyMap& self, PropertyMap& other, bp::list names) {
	bp::stl_input_iterator<std::string> begin(names), end;
	self.exposeTo(other, std::set<std::string>(begin, end));
}

}  // anonymous namespace

void export_properties() {
	bp::to_python_converter<PropertyMap::iterator::value_type, property_pair_to_python>();

#if 0  // TODO
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
	    .def("configureInitFrom", &PropertyMap_configureInitFrom, PropertyMap_configureInitFrom_overloads())
	    .def("exposeTo", &PropertyMap_exposeTo_1)
	    .def("exposeTo", &PropertyMap_exposeTo_2)
	    .def("exposeTo", &PropertyMap_exposeTo_l);
}
}
}
