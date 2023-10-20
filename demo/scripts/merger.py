#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit_commander.roscpp_initializer import roscpp_initialize
import time

roscpp_initialize("mtc_tutorial_merger")

# use the joint interpolation planner
planner = core.JointInterpolationPlanner()

# the task will contain our stages
task = core.Task()

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
