// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit::planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface);
}

namespace moveit::task_constructor::subtasks {

class Gripper : public SubTask {
public:
	Gripper(std::string name);

	virtual bool canCompute();

	virtual bool compute();

	void setGroup(std::string group);

	void setFrom(std::string named_target);
	void setTo(std::string named_target);

	void graspObject(std::string object);

protected:
	std::string group_;

	std::string from_named_target_;
	std::string to_named_target_;

	std::string object_;

	moveit::planning_interface::MoveGroupInterfacePtr mgi_;
};

}
