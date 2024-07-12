#! /usr/bin/env python3
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped
import rclcpp
import time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# Create a task
task = core.Task()
task.name = "modify planning scene"
task.loadRobotModel(node)

# Add the current state to the task hierarchy
task.add(stages.CurrentState("current state"))

# [initAndConfigModifyPlanningScene]
# Specify object parameters
object_name = "grasp_object"
object_radius = 0.02

objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.orientation.x = 1.0
objectPose.pose.position.x = 0.30702
objectPose.pose.position.y = 0.0
objectPose.pose.position.z = 0.285

object = CollisionObject()
object.header.frame_id = "world"
object.id = object_name
sphere = SolidPrimitive()
sphere.type = sphere.SPHERE
sphere.dimensions.insert(sphere.SPHERE_RADIUS, object_radius)

object.primitives.append(sphere)
object.primitive_poses.append(objectPose.pose)
object.operation = object.ADD

modifyPlanningScene = stages.ModifyPlanningScene("modify planning scene")
modifyPlanningScene.addObject(object)
task.add(modifyPlanningScene)
# [initAndConfigModifyPlanningScene]

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)
