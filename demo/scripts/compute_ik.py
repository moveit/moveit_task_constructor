#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header
import time

import rclcpp

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Specify the planning group
group = "panda_arm"

# Create a task
task = core.Task()
task.name = "compute IK"
task.loadRobotModel(node)

# Add a stage to retrieve the current state
task.add(stages.CurrentState("current state"))

# Add a planning stage connecting the generator stages
planner = core.PipelinePlanner(node)  # create default planning pipeline
task.add(stages.Connect("connect", [(group, planner)]))  # operate on group
del planner  # Delete PipelinePlanner when not explicitly needed anymore

# [propertyTut12]
# Add a Cartesian pose generator
generator = stages.GeneratePose("cartesian pose")
# [propertyTut12]
# Inherit PlanningScene state from "current state" stage
generator.setMonitoredStage(task["current state"])
# Configure target pose
# [propertyTut13]
pose = Pose(position=Point(z=0.2))
generator.pose = PoseStamped(header=Header(frame_id="panda_link8"), pose=pose)
# [propertyTut13]

# [initAndConfigComputeIk]
# Wrap Cartesian generator into a ComputeIK stage to yield a joint pose
computeIK = stages.ComputeIK("compute IK", generator)
computeIK.group = group  # Use the group-specific IK solver
# Which end-effector frame should reach the target?
computeIK.ik_frame = PoseStamped(header=Header(frame_id="panda_link8"))
computeIK.max_ik_solutions = 4  # Limit the number of IK solutions
# [propertyTut14]
props = computeIK.properties
# derive target_pose from child's solution
props.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])
# [propertyTut14]

# Add the stage to the task hierarchy
task.add(computeIK)
# [initAndConfigComputeIk]

if task.plan():
    task.publish(task.solutions[0])

time.sleep(1)  # sleep some time to allow C++ threads to publish their messages
