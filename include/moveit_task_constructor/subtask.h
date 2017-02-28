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

	const std::vector<InterfaceState>& getOut();
	const std::vector<InterfaceState>& getIn();
	const std::list<SubTrajectory>& getTrajectories();

	void addPredecessor(SubTaskConstPtr);
	void addSuccessor(SubTaskConstPtr);

protected:
	const std::string name_;

	// maintain raw pointers to predecessors to avoid pointer circles
	std::vector<const SubTask*> predecessors_;
	std::vector<SubTaskConstPtr> successors_;

	std::list<SubTrajectory> trajectories_;

	std::shared_ptr< std::vector<InterfaceState> > ins_;
	std::shared_ptr< std::vector<InterfaceState> > outs_;
};

}
