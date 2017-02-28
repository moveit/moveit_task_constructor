// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/storage.h>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>

#include <vector>
#include <list>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}

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

	void setPlanningScene(planning_scene::PlanningSceneConstPtr);

	void addPredecessor(SubTaskPtr);
	void addSuccessor(SubTaskPtr);

protected:
	void sendForward();
	void sendBackward();
	void sendBothWays(robot_trajectory::RobotTrajectoryPtr&, planning_scene::PlanningSceneConstPtr&);

	void connectBegin(InterfaceState&, SubTrajectory*);
	void connectEnd(InterfaceState&, SubTrajectory*);

	InterfaceState* newBegin(planning_scene::PlanningSceneConstPtr, SubTrajectory*);
	InterfaceState* newEnd(planning_scene::PlanningSceneConstPtr, SubTrajectory*);

	const std::string name_;

	// maintain raw pointers to predecessors to avoid pointer circles
	std::vector<SubTask*> predecessors_;
	std::vector<SubTaskPtr> successors_;

	std::list<SubTrajectory> trajectories_;

	std::list<InterfaceState> beginnings_;
	std::list<InterfaceState> endings_;

	planning_scene::PlanningSceneConstPtr scene_;
};

}
