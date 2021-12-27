#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit.python_tools import roscpp_init
from geometry_msgs.msg import PoseStamped
import time

roscpp_init("mtc_tutorial_compute_ik")

# Specify the planning group
group = "panda_arm"

# Create a frame at the origin of link8
ikFrame = PoseStamped()
ikFrame.header.frame_id = "panda_link8"

# Create a task
task = core.Task()

# Get the current robot state
currentState = stages.CurrentState("current state")

# Calculate the inverse kinematics for the current robot state
computeIK = stages.ComputeIK("compute IK", currentState)

# Specify the planning group that should be used for ik calculation
computeIK.group = group

# ik_frame references possible tip frames of the kinematic tree of
# the chosen planning group
computeIK.ik_frame = ikFrame
computeIK.target_pose = ikFrame

# Limit the number of inverse kinematics solutions to 4
computeIK.max_ik_solutions = 4

# Add the stage to the task hierarchy
task.add(computeIK)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
