#include "mainloop_processing.h"
#include <ros/console.h>

namespace moveit { namespace tools {

MainLoopProcessing::MainLoopProcessing(QObject *parent) : QObject(parent)
{
}

void MainLoopProcessing::addJob(const std::function<void()>& job)
{
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	jobs_.push_back(job);
}

void MainLoopProcessing::clear()
{
	jobs_.clear();
}

size_t MainLoopProcessing::numPending()
{
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	return jobs_.size();
}

void MainLoopProcessing::waitForAllJobs()
{
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	while (!jobs_.empty())
		idle_condition_.wait(ulock);
}

void MainLoopProcessing::executeJobs()
{
	boost::unique_lock<boost::mutex> ulock(jobs_mutex_);
	while (!jobs_.empty())
	{
		std::function<void()> fn = jobs_.front();
		jobs_.pop_front();
		ulock.unlock();
		try
		{
			fn();
		}
		catch (std::exception& ex)
		{
			ROS_ERROR("Exception caught executing main loop job: %s", ex.what());
		}
		ulock.lock();
	}
	idle_condition_.notify_all();
}

} }
