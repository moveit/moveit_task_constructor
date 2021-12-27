#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init
import time

roscpp_init("mtc_tutorial_current_state")

# Create a task
task = core.Task()

# Add the current state to the task hierarchy
task.add(stages.CurrentState("current state"))

# check for collisions and find corrections
fixCollisionObjects = stages.FixCollisionObjects("FixCollisionObjects")

# cut off length for collision fixing
fixCollisionObjects.max_penetration = 0.01

# Add the stage to the task hierarchy
task.add(fixCollisionObjects)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
