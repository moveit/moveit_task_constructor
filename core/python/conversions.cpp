#include "conversions.h"
#include <ros/serialization.h>

namespace moveit {
namespace python {

std::string rosMsgName(const std::string& python_type_name)
{
	static const std::string key = ".msg._";
	size_t pos = python_type_name.find(key, 0);
	if (pos == std::string::npos)
		return std::string();

	std::string result = python_type_name;
	result.erase(pos+1, key.length()-1);
	result[pos] = '/';
	return result;
}

std::string _serializeMsg(const boost::python::object& msg)
{
	boost::python::object StringIO = boost::python::import("StringIO");
	boost::python::object buf = StringIO.attr("StringIO")();
	msg.attr("serialize")(buf);
	return boost::python::extract<std::string>(buf.attr("getvalue")());
}

boost::python::api::object _deserializeMsg(const std::string& data, const std::string& python_type_name)
{
	size_t pos = python_type_name.find('/');
	if (pos == std::string::npos)
		throw std::runtime_error("Invalid ROS msg name: " + python_type_name);

	std::string module_name = python_type_name.substr(0, pos) + ".msg";
	boost::python::object module = boost::python::import(module_name.c_str());
	boost::python::object msg = module.attr(python_type_name.substr(pos+1).c_str())();
	msg.attr("deserialize")(boost::python::object(data));
	return msg;
}

} }
