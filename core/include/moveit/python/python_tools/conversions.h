#pragma once

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include <ros/serialization.h>

namespace moveit {
namespace python {

template <typename T>
std::vector<T> fromList(const boost::python::list& values)
{
	boost::python::stl_input_iterator<T> begin(values), end;
	return std::vector<T>(begin, end);
}

template <typename T>
boost::python::list toList(const std::vector<T>& v)
{
	boost::python::list l;
	for (const T& value : v)
		l.append(value);
	return l;
}


template <typename T>
std::map<std::string, T> fromDict(const boost::python::dict& values)
{
	std::map<std::string, T> m;
	for (boost::python::stl_input_iterator<boost::python::tuple> it(values.iteritems()), end; it != end; ++it) {
		const std::string& key = boost::python::extract<std::string>((*it)[0]);
		const T& value = boost::python::extract<T>((*it)[1]);
		m.insert(std::make_pair(key, value));
	}
	return m;
}

template <typename T>
boost::python::dict toDict(const std::map<std::string, T>& v)
{
	boost::python::dict d;
	for (const std::pair<std::string, T>& p : v)
		d[p.first] = p.second;
	return d;
}


/// convert a ROS msg to a string
template <typename T>
std::string serializeMsg(const T& msg)
{
	static_assert(sizeof(uint8_t) == sizeof(char), "Assuming char has same size as uint8_t");
	std::size_t size = ros::serialization::serializationLength(msg);
	std::string result(size, '\0');
	if (size)
	{
		ros::serialization::OStream stream(reinterpret_cast<uint8_t*>(&result[0]), size);
		ros::serialization::serialize(stream, msg);
	}
	return result;
}

/// convert a string to a ROS message
template <typename T>
void deserializeMsg(const std::string& data, T& msg)
{
	ros::serialization::IStream stream(const_cast<uint8_t*>(reinterpret_cast<const uint8_t*>(&data[0])), data.size());
	ros::serialization::deserialize(stream, msg);
}

/// retrieve name of ROS msg from object instance
std::string rosMsgName(PyObject* object);

/// non-templated base class for RosMsgConverter<T> providing common methods
class RosMsgConverterBase {
protected:
	/// Register type internally and return true if registered first time
	static bool insert(const boost::python::type_info& type_info, const std::string& ros_msg_name);

	/// Determine if python object can be converted into C++ msg type
	static void* convertible(PyObject* object);

	/// Convert a ROS message (from python) to a string
	static std::string fromPython(const boost::python::object& msg);

	/// Convert a string to a python ROS message of given type
	static PyObject* toPython(const std::string& data, const boost::python::type_info& type_info);
};

/// converter type to be registered with boost::python type conversion
/// https://sixty-north.com/blog/how-to-write-boost-python-type-converters.html
template <typename T>
struct RosMsgConverter : RosMsgConverterBase {
	/// constructor registers the type converter
	RosMsgConverter() {
		auto type_info = boost::python::type_id<T>();
		// register type internally
		if (insert(type_info, ros::message_traits::DataType<T>::value())) {
			// https://stackoverflow.com/questions/9888289/checking-whether-a-converter-has-already-been-registered
			const boost::python::converter::registration* reg = boost::python::converter::registry::query(type_info);
			if (!reg || !reg->m_to_python) {
				/// register type with boost::python converter system
				boost::python::converter::registry::push_back(&convertible, &construct, type_info);
				boost::python::to_python_converter<T, RosMsgConverter<T>>();
			}
		}
	}

	/// Conversion from Python object to C++ object, using pre-allocated memory block
	static void construct(PyObject* object,
	                      boost::python::converter::rvalue_from_python_stage1_data* data)
	{
		// serialize python msgs into string
		std::string serialized = fromPython(boost::python::object(boost::python::borrowed(object)));

		// Obtain a pointer to the memory block that the converter has allocated for the C++ type.
		void* storage = reinterpret_cast<boost::python::converter::rvalue_from_python_storage<T>*>(data)->storage.bytes;
		// Allocate the C++ type into the pre-allocated memory block, and assign its pointer to the converter's convertible variable.
		T* result = new (storage) T();
		data->convertible = result;

		// deserialize string into C++ msg
		deserializeMsg(serialized, *result);
	}

	/// Conversion from C++ object to Python object
	static PyObject* convert(const T& msg) {
		return toPython(serializeMsg(msg), typeid(T));
	}
};

} }
