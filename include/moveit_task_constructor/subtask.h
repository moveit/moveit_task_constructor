// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/storage.h>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>

#include <vector>
#include <list>

namespace moveit::task_constructor {


MOVEIT_CLASS_FORWARD(SubTask);

class SubTask {
public:
	SubTask(std::string name);

	virtual bool canCompute() = 0;
	virtual bool compute() = 0;

	const std::string& getName();
	const std::list<InterfaceState>& getEnd();
	const std::list<InterfaceState>& getBegin();
	const std::list<SubTrajectory>& getTrajectories();

	void setRobotModel(robot_model::RobotModelConstPtr);

	void addPredecessor(SubTaskPtr);
	void addSuccessor(SubTaskPtr);

protected:
	void sendForward();
	void sendBackward();
	void sendBothWays(robot_trajectory::RobotTrajectoryPtr&, planning_scene::PlanningScenePtr&);

	void connectBegin(InterfaceState&, SubTrajectory*);
	void connectEnd(InterfaceState&, SubTrajectory*);

	InterfaceState* newBegin(planning_scene::PlanningScenePtr, SubTrajectory*);
	InterfaceState* newEnd(planning_scene::PlanningScenePtr, SubTrajectory*);

	const std::string name_;

	// maintain raw pointers to predecessors to avoid pointer circles
	std::vector<SubTask*> predecessors_;
	std::vector<SubTaskPtr> successors_;

	std::list<SubTrajectory> trajectories_;

	std::list<InterfaceState> beginnings_;
	std::list<InterfaceState> endings_;

	robot_model::RobotModelConstPtr robot_model_;
};

}
