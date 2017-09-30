// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace subtasks {

class Move : public Connecting {
public:
	Move(std::string name);

	virtual bool canCompute();

	virtual bool compute();

	void setGroup(std::string group);
	void setLink(std::string link);

	void setPlannerId(std::string planner);
	void setTimeout(double timeout);

	void setFrom(std::string named_target);
	void setTo(std::string named_target);

protected:
	std::string group_;
	std::string link_;

	double timeout_;

	std::string planner_id_;

	std::string from_named_target_;
	std::string to_named_target_;

	moveit::planning_interface::MoveGroupInterfacePtr mgi_;
};

} } }
