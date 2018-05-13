#pragma once

#include <boost/python.hpp>
#include <boost/python/stl_iterator.hpp>
#include <string>
#include <vector>
#include <map>
#include <memory>

#include <ros/serialization.h>

/// Tag std::shared_ptr and std::unique_ptr as smart pointers
namespace boost {

template<class T>
T* get_pointer(std::shared_ptr<T> p) { return p.get(); }

template<class T>
T* get_pointer(std::unique_ptr<T> p) { return p.get(); }

namespace python {

template <class T> struct pointee<std::shared_ptr<T> > { typedef T type; };
//template <class T> struct pointee<std::unique_ptr<T> > { typedef T type; };

} }


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


/// convert a ROS message (from python) to a string
std::string _serializeMsg(const boost::python::object& msg);

/// convert a string to a python ROS message of given type
boost::python::object _deserializeMsg(const std::string& data, const std::string& python_type_name);


/// convert a python type name (extracted from __class__.__module__) to form package/msg
std::string rosMsgName(const std::string& python_type_name);

/// convert a ROS message from python to C++
template <typename T>
T fromPython(const boost::python::object &o)
{
	std::string serialized = _serializeMsg(o);
	T result;
	deserializeMsg(serialized, result);
	return result;
}

/// convert a ROS message from C++ to python
template <typename T>
boost::python::object toPython(const std::string& python_type_name, const T& msg)
{
	std::string serialized = serializeMsg(msg);
	return _deserializeMsg(serialized, python_type_name);
}

} }
