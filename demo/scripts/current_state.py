#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Create a task
task = core.Task()
task.name = "current state"
task.loadRobotModel(node)

# Get the current robot state
currentState = stages.CurrentState("current state")

# Add the stage to the task hierarchy
task.add(currentState)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
