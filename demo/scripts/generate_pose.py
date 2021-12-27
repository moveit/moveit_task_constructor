#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped
from moveit.python_tools import roscpp_init
import time

roscpp_init("mtc_tutorial_compute_ik")

# Specify the planning group
group = "panda_arm"

# Create a task
task = core.Task()

# Get the current robot state
currentState = stages.CurrentState("current state")
task.add(currentState)

# Create a planner instance that is used to connect
# the current state to the grasp approach pose
pipelinePlanner = core.PipelinePlanner()
pipelinePlanner.planner = "RRTConnectkConfigDefault"
planners = [(group, pipelinePlanner)]

# Connect the two stages
connect = stages.Connect("connect1", planners)
connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
task.add(connect)

# create an example pose wrt. the origin of the
# panda arm link8
pose = PoseStamped()
pose.header.frame_id = "panda_link8"

# Calculate the inverse kinematics for the current robot state
generatePose = stages.GeneratePose("generate pose")

# spwan a pose whenever there is a solution of the monitored stage
generatePose.setMonitoredStage(task["current state"])
generatePose.pose = pose

# Add the stage to the task hierarchy
task.add(generatePose)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
