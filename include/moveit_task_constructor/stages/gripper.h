// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/stage.h>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace stages {

class Gripper : public PropagatingEitherWay {
public:
	Gripper(std::string name);

	bool init(const planning_scene::PlanningSceneConstPtr &scene);
	bool computeForward(const InterfaceState& from) override;
	bool computeBackward(const InterfaceState& to) override;

	void setEndEffector(std::string eef);
	void setAttachLink(std::string link);

	void setFrom(std::string named_target);
	void setTo(std::string named_target);

	void graspObject(std::string grasp_object);

protected:
	bool compute(const InterfaceState &state, planning_scene::PlanningScenePtr &scene,
	             robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost, Direction dir);

protected:
	std::string eef_;
	std::string named_target_;
	std::string grasp_object_;
	std::string attach_link_;

	planning_pipeline::PlanningPipelinePtr planner_;
	moveit::planning_interface::MoveGroupInterfacePtr mgi_;
};

} } }
