// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace task_constructor { namespace subtasks {

class CurrentState : public Generator {
public:
	CurrentState(std::string name);

	virtual bool canCompute();

	virtual bool compute();

protected:
	bool ran_;
};

} } }
