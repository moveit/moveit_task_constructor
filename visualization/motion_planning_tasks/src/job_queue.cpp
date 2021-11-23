/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld University
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

/* Author: Robert Haschke */

#include "job_queue.h"
#include <rclcpp/logging.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_task_constructor_visualization.job_queue");

namespace moveit {
namespace tools {

JobQueue::JobQueue(QObject* parent) : QObject(parent) {}

void JobQueue::addJob(const std::function<void()>& job) {
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	jobs_.push_back(job);
}

void JobQueue::clear() {
	jobs_.clear();
}

size_t JobQueue::numPending() {
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	return jobs_.size();
}

void JobQueue::waitForAllJobs() {
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	while (!jobs_.empty())
		idle_condition_.wait(ulock);
}

void JobQueue::executeJobs() {
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	while (!jobs_.empty()) {
		std::function<void()> fn = jobs_.front();
		jobs_.pop_front();
		ulock.unlock();
		try {
			fn();
		} catch (std::exception& ex) {
			RCLCPP_ERROR(LOGGER, "Exception caught executing main loop job: %s", ex.what());
		}
		ulock.lock();
	}
	idle_condition_.notify_all();
}
}  // namespace tools
}  // namespace moveit
