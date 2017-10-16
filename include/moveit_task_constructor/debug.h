#pragma once

#include <moveit/macros/class_forward.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <ros/publisher.h>
#include <set>

namespace moveit { namespace task_constructor {

MOVEIT_CLASS_FORWARD(Task)
MOVEIT_CLASS_FORWARD(SubTrajectory)

bool publishSolution(ros::Publisher& pub, moveit_msgs::DisplayTrajectory& dt,
                     const std::vector<const moveit::task_constructor::SubTrajectory *> &solution, bool wait);

void publishAllPlans(const Task &task, const std::string &topic = "task_plan", bool wait = true);

class NewSolutionPublisher {
   std::set<const SubTrajectory*> published_;
   const Task &task_;
   ros::Publisher pub_;

public:
   NewSolutionPublisher(const Task &task, const std::string &topic = "task_plan");
   void publish();
};

} }
