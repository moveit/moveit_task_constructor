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

co.id = "object"
co.primitives[0].type = SolidPrimitive.BOX
co.primitives[0].dimensions = [0.1, 0.05, 0.03]
pose = co.primitive_poses[0]
pose.position.x = 0.30702
pose.position.y = 0.0
pose.position.z = 0.485
pose.orientation.x = pose.orientation.w = 0.70711  # 90° about x
mps.addObject(co)
mps.attachObjects(["object"], "panda_hand")

task.add(mps)

move = stages.MoveRelative("y +0.4", planner)
move.timeout = 5
move.group = group
header = Header(frame_id="world")
move.setDirection(Vector3Stamped(header=header, vector=Vector3(x=0.0, y=0.4, z=0.0)))

constraints = Constraints()
oc = OrientationConstraint()
oc.parameterization = oc.ROTATION_VECTOR
oc.header.frame_id = "world"
oc.link_name = "object"
oc.orientation.x = oc.orientation.w = 0.70711  # 90° about x
oc.absolute_x_axis_tolerance = 0.1
oc.absolute_y_axis_tolerance = 0.1
oc.absolute_z_axis_tolerance = 3.14
oc.weight = 1.0
constraints.orientation_constraints.append(oc)
move.path_constraints = constraints

task.add(move)

if task.plan():
    task.publish(task.solutions[0])
time.sleep(100)
