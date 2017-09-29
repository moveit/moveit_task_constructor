// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace subtasks {

class Gripper : public PropagatingForward {
public:
	Gripper(std::string name);

	virtual bool canCompute();

	virtual bool compute();

	void setEndEffector(std::string eef);

	void setAttachLink(std::string link);

	void setFrom(std::string named_target);
	void setTo(std::string named_target);

	void graspObject(std::string grasp_object);

protected:
	std::string eef_;

	std::string from_named_target_;
	std::string to_named_target_;

	std::string grasp_object_;

	moveit::planning_interface::MoveGroupInterfacePtr mgi_;

	std::string attach_link_;
};

} } }
