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
