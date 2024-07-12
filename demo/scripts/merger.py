#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# use the joint interpolation planner
planner = core.JointInterpolationPlanner()

# the task will contain our stages
task = core.Task()
task.name = "merger"
task.loadRobotModel(node)

# start from current robot state
currentState = stages.CurrentState("current state")
task.add(currentState)

# [initAndConfigMerger]
# the merger plans for two parallel execution paths
merger = core.Merger("Merger")

# first simultaneous execution
moveTo1 = stages.MoveTo("Move To Home", planner)
moveTo1.group = "hand"
moveTo1.setGoal("close")
merger.insert(moveTo1)

# second simultaneous execution
moveTo2 = stages.MoveTo("Move To Ready", planner)
moveTo2.group = "panda_arm"
moveTo2.setGoal("extended")
merger.insert(moveTo2)

# add the merger stage to the task hierarchy
task.add(merger)
# [initAndConfigMerger]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
