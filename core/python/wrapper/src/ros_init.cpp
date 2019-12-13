#include <moveit/python/python_tools/ros_init.h>
#include <ros/init.h>

namespace moveit {
namespace python {

boost::mutex InitProxy::lock;
std::unique_ptr<InitProxy> InitProxy::singleton_instance;

void InitProxy::init(const std::string& node_name, const std::map<std::string, std::string>& remappings,
                     uint32_t options) {
	boost::mutex::scoped_lock slock(lock);
	if (!singleton_instance && !ros::isInitialized())
		singleton_instance.reset(new InitProxy(node_name, remappings, options));
}

void InitProxy::shutdown() {
	boost::mutex::scoped_lock slock(lock);
	singleton_instance.reset();
}

InitProxy::InitProxy(const std::string& node_name, const std::map<std::string, std::string>& remappings,
                     uint32_t options) {
	ros::init(remappings, node_name, options | ros::init_options::NoSigintHandler);
	spinner.reset(new ros::AsyncSpinner(1));
	spinner->start();
}

InitProxy::~InitProxy() {
	spinner->stop();
	spinner.reset();
}
}  // namespace python
}  // namespace moveit
