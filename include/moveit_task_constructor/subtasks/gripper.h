// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/subtask.h>

namespace moveit { namespace planning_interface {
MOVEIT_CLASS_FORWARD(MoveGroupInterface)
} }

namespace moveit { namespace task_constructor { namespace subtasks {

class Gripper : public PropagatingAnyWay {
public:
	Gripper(std::string name);

	bool compute(const InterfaceState &state, planning_scene::PlanningScenePtr &scene,
	             robot_trajectory::RobotTrajectoryPtr &trajectory, double &cost, Direction dir);
	bool computeForward(const InterfaceState& from, planning_scene::PlanningScenePtr &to,
	                    robot_trajectory::RobotTrajectoryPtr& trajectory, double& cost) override;
	bool computeBackward(planning_scene::PlanningScenePtr& from, const InterfaceState& to,
	                     robot_trajectory::RobotTrajectoryPtr& trajectory, double& cost) override;

	void setEndEffector(std::string eef);
	void setAttachLink(std::string link);

	void setFrom(std::string named_target);
	void setTo(std::string named_target);

	void graspObject(std::string grasp_object);

protected:
	std::string eef_;
	std::string named_target_;
	std::string grasp_object_;
	std::string attach_link_;

	moveit::planning_interface::MoveGroupInterfacePtr mgi_;
};

} } }
