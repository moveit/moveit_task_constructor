#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.core import planning_scene
from moveit.task_constructor import core, stages
import rclcpp
from moveit.core.planning_scene import PlanningScene
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Create a task
task = core.Task()
task.name = "fixed state"

# [initAndConfigFixedState]
# Initialize a PlanningScene for use in a FixedState stage
task.loadRobotModel(node)  # load the robot model (usually done in init())
planningScene = PlanningScene(task.getRobotModel())

# Create a FixedState stage and pass the created PlanningScene as its state
fixedState = stages.FixedState("fixed state")
fixedState.setState(planningScene)

# Add the stage to the task hierarchy
task.add(fixedState)
# [initAndConfigFixedState]

if task.plan():
    task.publish(task.solutions[0])

del planningScene  # Avoid ClassLoader warning by destroying the RobotModel
time.sleep(1)
