#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Use the joint interpolation planner
jointPlanner = core.JointInterpolationPlanner()

# Create a task
task = core.Task()
task.name = "alternatives"
task.loadRobotModel(node)

# Start from current robot state
currentState = stages.CurrentState("current state")

# Add the current state to the task hierarchy
task.add(currentState)

# [initAndConfigAlternatives]
# The alternatives stage supports multiple execution paths
alternatives = core.Alternatives("Alternatives")

# goal 1
goalConfig1 = {
    "panda_joint1": 1.0,
    "panda_joint2": -1.0,
    "panda_joint3": 0.0,
    "panda_joint4": -2.5,
    "panda_joint5": 1.0,
    "panda_joint6": 1.0,
    "panda_joint7": 1.0,
}

# goal 2
goalConfig2 = {
    "panda_joint1": -3.0,
    "panda_joint2": -1.0,
    "panda_joint3": 0.0,
    "panda_joint4": -2.0,
    "panda_joint5": 1.0,
    "panda_joint6": 2.0,
    "panda_joint7": 0.5,
}

# First motion plan to compare
moveTo1 = stages.MoveTo("Move To Goal Configuration 1", jointPlanner)
moveTo1.group = "panda_arm"
moveTo1.setGoal(goalConfig1)
alternatives.insert(moveTo1)

# Second motion plan to compare
moveTo2 = stages.MoveTo("Move To Goal Configuration 2", jointPlanner)
moveTo2.group = "panda_arm"
moveTo2.setGoal(goalConfig2)
alternatives.insert(moveTo2)

# Add the alternatives stage to the task hierarchy
task.add(alternatives)
# [initAndConfigAlternatives]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
