#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit_commander.roscpp_initializer import roscpp_initialize
import time

roscpp_initialize("mtc_tutorial_current_state")

# Create a task
task = core.Task()

# Get the current robot state
currentState = stages.CurrentState("current state")

# Add the stage to the task hierarchy
task.add(currentState)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
