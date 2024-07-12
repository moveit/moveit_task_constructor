#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Create a task
task = core.Task()
task.name = "fix collision objects"
task.loadRobotModel(node)

# Add the current state to the task hierarchy
task.add(stages.CurrentState("current state"))

# [initAndConfig]
# check for collisions and find corrections
fixCollisionObjects = stages.FixCollisionObjects("FixCollisionObjects")

# cut off length for collision fixing
fixCollisionObjects.max_penetration = 0.01

# Add the stage to the task hierarchy
task.add(fixCollisionObjects)
# [initAndConfig]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
