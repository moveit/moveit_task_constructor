#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ros/publisher.h>
#include <set>

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SolutionBase)

void publishAllPlans(const Task &task, const std::string &topic = "task_plan", bool wait = true);

class NewSolutionPublisher {
   ros::Publisher pub_;

public:
   NewSolutionPublisher(const std::string &topic = "task_plan");
   void operator()(const SolutionBase &s);
};

} }
