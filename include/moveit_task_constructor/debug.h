#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/DisplayTrajectory.h>

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SubTrajectory)

bool publishSolution(ros::Publisher& pub, moveit_msgs::DisplayTrajectory& dt,
                     std::vector<SubTrajectory*>& solution, bool wait);

void publishAllPlans(const Task &task, const std::string &topic = "task_plan", bool wait = true);

} }
