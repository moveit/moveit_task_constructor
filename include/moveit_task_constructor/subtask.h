// copyright Michael 'v4hn' Goerner @ 2017

#pragma once

#include <moveit_task_constructor/storage.h>

#include <moveit/macros/class_forward.h>
#include <moveit/planning_scene/planning_scene.h>

#include <vector>
#include <list>
#include <tuple>

namespace planning_scene {
MOVEIT_CLASS_FORWARD(PlanningScene);
}

namespace planning_pipeline {
MOVEIT_CLASS_FORWARD(PlanningPipeline);
}

namespace moveit::task_constructor {

MOVEIT_CLASS_FORWARD(SubTask);

class SubTask {
public:
	SubTask(std::string name);

	virtual bool canCompute() = 0;
	virtual bool compute() = 0;

	const std::string& getName();
	const std::list<InterfaceState>& getBegin();
	const std::list<InterfaceState>& getEnd();
	const std::list<SubTrajectory>& getTrajectories();

	void setPlanningScene(planning_scene::PlanningSceneConstPtr);
	void setPlanningPipeline(planning_pipeline::PlanningPipelinePtr);

	void addPredecessor(SubTaskPtr);
	void addSuccessor(SubTaskPtr);

protected:
	InterfaceState& fetchStateBeginning();
	InterfaceState& fetchStateEnding();
	std::pair<InterfaceState&, InterfaceState&> fetchStatePair();

	bool hasBeginning();
	bool hasEnding();
	bool hasStatePair();

	SubTrajectory& addTrajectory(robot_trajectory::RobotTrajectoryPtr);

	void sendForward(SubTrajectory&, planning_scene::PlanningSceneConstPtr);
	void sendBackward(SubTrajectory&, planning_scene::PlanningSceneConstPtr);
	void sendBothWays(SubTrajectory&, planning_scene::PlanningSceneConstPtr);

	InterfaceState* newBegin(planning_scene::PlanningSceneConstPtr, SubTrajectory*);
	InterfaceState* newEnd(planning_scene::PlanningSceneConstPtr, SubTrajectory*);

	const std::string name_;

	// maintain raw pointers to predecessors to avoid pointer circles
	SubTask* predecessor_;
	SubTaskPtr successor_;

	std::list<SubTrajectory> trajectories_;

	std::list<InterfaceState> beginnings_;
	std::list<InterfaceState> endings_;

	planning_scene::PlanningSceneConstPtr scene_;
	planning_pipeline::PlanningPipelinePtr planner_;

	std::list<InterfaceState>::iterator it_beginnings_;
	std::list<InterfaceState>::iterator it_endings_;

	std::pair< std::list<InterfaceState>::iterator, std::list<InterfaceState>::iterator > it_pairs_;
};

}
