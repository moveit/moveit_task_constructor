// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/stage.h>

namespace moveit {
namespace planning_interface { MOVEIT_CLASS_FORWARD(MoveGroupInterface)}
}

namespace moveit { namespace task_constructor { namespace stages {

class Move : public Connecting {
public:
	Move(std::string name);

	void init(const planning_scene::PlanningSceneConstPtr &scene);
	bool compute(const InterfaceState &from, const InterfaceState &to);

	void setGroup(std::string group);
	void setPlannerId(std::string planner);
	void setTimeout(double timeout);

protected:
	std::string group_;
	std::string planner_id_;
	double timeout_;

	planning_pipeline::PlanningPipelinePtr planner_;
	moveit::planning_interface::MoveGroupInterfacePtr mgi_;
};

} } }
