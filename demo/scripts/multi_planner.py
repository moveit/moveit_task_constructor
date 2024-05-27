#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit_commander.roscpp_initializer import roscpp_initialize
import time

roscpp_initialize("mtc_tutorial_multi_planner")

ompl_pipelinePlanner = core.PipelinePlanner("ompl")
ompl_pipelinePlanner.planner = "RRTConnectkConfigDefault"
pilz_pipelinePlanner = core.PipelinePlanner("pilz_industrial_motion_planner")
pilz_pipelinePlanner.planner = "PTP"
multiPlanner = core.MultiPlanner()
multiPlanner.add(ompl_pipelinePlanner)
multiPlanner.add(pilz_pipelinePlanner)

# Create a task
task = core.Task()

# Start from current robot state
currentState = stages.CurrentState("current state")

# Add the current state to the task hierarchy
task.add(currentState)

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
moveTo1 = stages.MoveTo("Move To Goal Configuration 1", multiPlanner)
moveTo1.group = "panda_arm"
moveTo1.setGoal(goalConfig1)
alternatives.insert(moveTo1)

# Second motion plan to compare
moveTo2 = stages.MoveTo("Move To Goal Configuration 2", multiPlanner)
moveTo2.group = "panda_arm"
moveTo2.setGoal(goalConfig2)
alternatives.insert(moveTo2)

# Add the alternatives stage to the task hierarchy
task.add(alternatives)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)