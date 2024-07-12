#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from math import pi
import time

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3
from moveit.task_constructor import core, stages

import rclcpp

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# [cartesianTut1]
group = "panda_arm"
# [cartesianTut1]

# [cartesianTut2]
# Cartesian and joint-space interpolation planners
cartesian = core.CartesianPath()
jointspace = core.JointInterpolationPlanner()
# [cartesianTut2]

# [cartesianTut3]
task = core.Task()
task.name = "cartesian"
task.loadRobotModel(node)

# start from current robot state
task.add(stages.CurrentState("current state"))
# [cartesianTut3]

# [initAndConfigMoveRelative]
# move along x
move = stages.MoveRelative("x +0.2", cartesian)
move.group = group
header = Header(frame_id="world")
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.2, y=0.0, z=0.0)))
task.add(move)
# [initAndConfigMoveRelative]

# [cartesianTut4]
# move along y
move = stages.MoveRelative("y -0.3", cartesian)
move.group = group
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.0, y=-0.3, z=0.0)))
task.add(move)
# [cartesianTut4]

# [cartesianTut5]
# rotate about z
move = stages.MoveRelative("rz +45°", cartesian)
move.group = group
move.setDirection(
    TwistStamped(header=header, twist=Twist(angular=Vector3(x=0.0, y=0.0, z=pi / 4.0)))
)
task.add(move)
# [cartesianTut5]

# [cartesianTut6]
# Cartesian motion, defined as joint-space offset
move = stages.MoveRelative("joint offset", cartesian)
move.group = group
move.setDirection(dict(panda_joint1=pi / 6, panda_joint3=-pi / 6))
task.add(move)
# [cartesianTut6]

# [initAndConfigMoveTo]
# moveTo named posture, using joint-space interplation
move = stages.MoveTo("moveTo ready", jointspace)
move.group = group
move.setGoal("ready")
task.add(move)
# [initAndConfigMoveTo]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(100)
