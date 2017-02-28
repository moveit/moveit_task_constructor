// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit::task_constructor::subtasks {

class CurrentState : public SubTask {
public:
	using SubTask::SubTask;

	virtual bool canCompute();

	virtual bool compute();

protected:
	bool succeeded_;
};

}
