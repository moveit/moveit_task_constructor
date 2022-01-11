#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose, Vector3
from std_msgs.msg import Header
import time

from moveit.python_tools import roscpp_init

roscpp_init("mtc_tutorial_compute_ik")

# Specify the planning group
group = "panda_arm"

# Create a task
task = core.Task()

# Add a stage to retrieve the current state
task.add(stages.CurrentState("current state"))

# Add a planning stage connecting the generator stages
planner = core.PipelinePlanner()  # create default planning pipeline
task.add(stages.Connect("connect", [(group, planner)]))  # operate on group
del planner  # Delete PipelinePlanner when not explicitly needed anymore

# Add a Cartesian pose generator
generator = stages.GeneratePose("cartesian pose")
# Inherit PlanningScene state from "current state" stage
generator.setMonitoredStage(task["current state"])
# Configure target pose
pose = Pose(position=Vector3(z=0.2))
generator.pose = PoseStamped(header=Header(frame_id="panda_link8"), pose=pose)

# Wrap Cartesian generator into a ComputeIK stage to yield a joint pose
computeIK = stages.ComputeIK("compute IK", generator)
computeIK.group = group  # Use the group-specific IK solver
computeIK.ik_frame = "panda_link8"  # Which end-effector frame should reach the target?
computeIK.max_ik_solutions = 4  # Limit the number of IK solutions
props = computeIK.properties
# derive target_pose from child's solution
props.configureInitFrom(core.Stage.PropertyInitializerSource.INTERFACE, ["target_pose"])

# Add the stage to the task hierarchy
task.add(computeIK)

if task.plan():
    task.publish(task.solutions[0])

time.sleep(1)  # sleep some time to allow C++ threads to publish their messages
