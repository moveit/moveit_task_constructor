#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.core import planning_scene
from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init
from moveit_commander import PlanningScene
import time

roscpp_init("mtc_tutorial_current_state")


# Create a task
task = core.Task()

# Get the current robot state
fixedState = stages.FixedState("fixed state")

# create an empty planning scene and assign it to the
# fixed state
planningScene = PlanningScene()
fixedState.setState(planningScene)

# Add the stage to the task hierarchy
task.add(fixedState)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
