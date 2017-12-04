// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit/task_constructor/stage.h>

namespace moveit { namespace task_constructor { namespace stages {

class Move : public Connecting {
public:
	Move(std::string name);

	void init(const planning_scene::PlanningSceneConstPtr &scene);
	bool compute(const InterfaceState &from, const InterfaceState &to);

	void setGroup(const std::__cxx11::string &group);
	void setPlannerId(const std::__cxx11::string &planner);
	void setTimeout(double timeout);

protected:
	planning_pipeline::PlanningPipelinePtr planner_;
};

} } }
