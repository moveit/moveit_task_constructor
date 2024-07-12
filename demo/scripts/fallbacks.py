#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# use cartesian and joint interpolation planners
cartesianPlanner = core.CartesianPath()
jointPlanner = core.JointInterpolationPlanner()

# initialize the mtc task
task = core.Task()
task.name = "fallbacks"
task.loadRobotModel(node)

# add the current planning scene state to the task hierarchy
currentState = stages.CurrentState("Current State")
task.add(currentState)

# [initAndConfigFallbacks]
# create a fallback container to fall back to a different planner
# if motion generation fails with the primary one
fallbacks = core.Fallbacks("Fallbacks")

# primary motion plan
moveTo1 = stages.MoveTo("Move To Goal Configuration 1", cartesianPlanner)
moveTo1.group = "panda_arm"
moveTo1.setGoal("extended")
fallbacks.insert(moveTo1)

# fallback motion plan
moveTo2 = stages.MoveTo("Move To Goal Configuration 2", jointPlanner)
moveTo2.group = "panda_arm"
moveTo2.setGoal("extended")
fallbacks.insert(moveTo2)

# add the fallback container to the task hierarchy
task.add(fallbacks)
# [initAndConfigFallbacks]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
