#pragma once
#include <QObject>

#include <QObject>
#include <deque>
#include <functional>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace moveit { namespace tools {

class MainLoopProcessing : public QObject
{
	Q_OBJECT
	boost::mutex jobs_mutex_;
	std::deque<std::function<void()> > jobs_;
	boost::condition_variable idle_condition_;

public:
	explicit MainLoopProcessing(QObject *parent = 0);
	void addJob(const std::function<void()> &job);
	void clear();
	size_t numPending();

	void waitForAllJobs();
	void executeJobs();
};

} }
