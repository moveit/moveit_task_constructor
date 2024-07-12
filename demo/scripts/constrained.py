#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Vector3, Pose
from moveit_msgs.msg import CollisionObject, Constraints, OrientationConstraint
from shape_msgs.msg import SolidPrimitive
from moveit.task_constructor import core, stages
import time

import rclcpp

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

group = "panda_arm"
planner = core.PipelinePlanner(node)

task = core.Task()
task.name = "constrained"
task.loadRobotModel(node)

task.add(stages.CurrentState("current state"))

co = CollisionObject()
co.header.frame_id = "world"
co.id = "obstacle"
sphere = SolidPrimitive()
sphere.type = sphere.SPHERE
sphere.dimensions.insert(sphere.SPHERE_RADIUS, 0.1)

pose = Pose()
pose.position.x = 0.3
pose.position.y = 0.2
pose.position.z = 0.5
pose.orientation.w = 1.0
co.primitives.append(sphere)
co.primitive_poses.append(pose)
co.operation = co.ADD

mps = stages.ModifyPlanningScene("modify planning scene")
mps.addObject(co)
task.add(mps)

move = stages.MoveRelative("y +0.4", planner)
move.group = group
header = Header(frame_id="world")
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.0, y=0.4, z=0.0)))

constraints = Constraints()
oc = OrientationConstraint()
oc.header.frame_id = "world"
oc.link_name = "panda_hand"
oc.orientation.x = 1.0
oc.orientation.w = 0.0
oc.absolute_x_axis_tolerance = 0.01
oc.absolute_y_axis_tolerance = 0.01
oc.absolute_z_axis_tolerance = 0.01
oc.weight = 1.0
constraints.orientation_constraints.append(oc)
move.path_constraints = constraints

task.add(move)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(100)
