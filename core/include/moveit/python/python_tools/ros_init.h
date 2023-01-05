#pragma once

#include <memory>
#include <boost/thread/mutex.hpp>
#include <ros/spinner.h>

namespace moveit {
namespace python {

/// singleton class to initialize ROS C++ (once) from Python
class InitProxy
{
public:
	static void init(const std::string& node_name = "moveit_python_wrapper",
	                 const std::map<std::string, std::string>& remappings = std::map<std::string, std::string>(),
	                 uint32_t options = 0);
	static void shutdown();

	~InitProxy();

private:
	InitProxy(const std::string& node_name, const std::map<std::string, std::string>& remappings, uint32_t options);

	static boost::mutex lock_;
	static std::unique_ptr<InitProxy> singleton_instance_;

private:
	std::unique_ptr<ros::AsyncSpinner> spinner;
};
}  // namespace python
}  // namespace moveit
