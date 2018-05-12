#pragma once

#include <memory>
#include <boost/python.hpp>
#include <boost/thread/mutex.hpp>
#include <ros/init.h>

namespace moveit {
namespace python {

class InitProxy {
public:
	static void init(const std::string& node_name="moveit_python_wrapper",
	                 const boost::python::dict& remappings = boost::python::dict());
	static void shutdown();

	~InitProxy();

private:
	InitProxy(const std::string& node_name, const boost::python::dict& remappings);

	static boost::mutex lock;
	static std::unique_ptr<InitProxy> singleton_instance;

private:
	std::unique_ptr<ros::AsyncSpinner> spinner;
};

} }
