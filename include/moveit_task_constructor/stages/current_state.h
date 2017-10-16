// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/stage.h>

namespace moveit { namespace task_constructor { namespace stages {

class CurrentState : public Generator {
public:
	CurrentState(std::string name);

	bool init(const planning_scene::PlanningSceneConstPtr& scene) override;
	bool canCompute() const override;
	bool compute() override;

protected:
	planning_scene::PlanningSceneConstPtr scene_;
	bool ran_;
};

} } }
