#pragma once

#include <moveit/python/python_tools/ros_types.h>
#include <moveit/python/python_tools/geometry_msg_types.h>
#include <moveit/task_constructor/properties.h>
#include <boost/any.hpp>
#include <typeindex>

namespace moveit {
namespace python {

class PYBIND11_EXPORT PropertyConverterBase
{
public:
	typedef pybind11::object (*to_python_converter_function)(const boost::any&);
	typedef boost::any (*from_python_converter_function)(const pybind11::object&);

protected:
	static bool insert(const std::type_index& type_index, const std::string& ros_msg_name,
	                   to_python_converter_function to, from_python_converter_function from);
};

/// utility class to register C++ / Python converters for a property of type T
template <typename T>
class PropertyConverter : protected PropertyConverterBase
{
public:
	PropertyConverter() { insert(typeid(T), rosMsgName<T>(), &toPython, &fromPython); }

private:
	static pybind11::object toPython(const boost::any& value) { return pybind11::cast(boost::any_cast<T>(value)); }
	static boost::any fromPython(const pybind11::object& po) { return pybind11::cast<T>(po); }

	template <class Q = T>
	typename std::enable_if<ros::message_traits::IsMessage<Q>::value, std::string>::type rosMsgName() {
		return ros::message_traits::DataType<T>::value();
	}

	template <class Q = T>
	typename std::enable_if<!ros::message_traits::IsMessage<Q>::value, std::string>::type rosMsgName() {
		return std::string();
	}
};

namespace properties {

/** Extension for pybind11::class_ to allow convienient definition of properties
 *
 * New method property<PropertyType>(const char* name) adds a property getter/setter.
 */

template <typename type_, typename... options>
class class_ : public pybind11::classh<type_, options...>
{
	using base_class_ = pybind11::classh<type_, options...>;

public:
	// forward all constructors
	using base_class_::classh;

	template <typename PropertyType, typename... Extra>
	class_& property(const char* name, const Extra&... extra) {
		PropertyConverter<PropertyType>();  // register corresponding property converter
		auto getter = [name](const type_& self) {
			const moveit::task_constructor::PropertyMap& props = self.properties();
			return props.get<PropertyType>(name);
		};
		auto setter = [name](type_& self, const PropertyType& value) {
			moveit::task_constructor::PropertyMap& props = self.properties();
			props.set(name, boost::any(value));
		};
		base_class_::def_property(name, getter, setter, pybind11::return_value_policy::reference_internal, extra...);
		return *this;
	}
};
}  // namespace properties
}  // namespace python
}  // namespace moveit
