#pragma once

#include <memory>
#include <boost/python.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/init.h>

namespace moveit {
namespace python {

/// singleton class to initialize ROS C++ (once) from Python
class InitProxy {
public:
	static void init(const std::string& node_name="moveit_python_wrapper",
	                 const boost::python::dict& remappings = boost::python::dict(),
	                 uint32_t options = 0);
	static void shutdown();

	~InitProxy();

private:
	InitProxy(const std::string& node_name, const boost::python::dict& remappings, uint32_t options);

	static boost::mutex lock;
	static std::unique_ptr<InitProxy> singleton_instance;

private:
	std::unique_ptr<ros::AsyncSpinner> spinner;
};

} }
