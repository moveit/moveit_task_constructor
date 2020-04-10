#! /usr/bin/env python
# -*- coding: utf-8 -*-

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist, Vector3Stamped, Vector3, PoseStamped, Pose, Point, Quaternion
from moveit.task_constructor import core, stages
import moveit_commander
import rospy
import numpy

from moveit.python_tools import roscpp_init
roscpp_init("mtc_tutorial")
rospy.init_node('mtc_tutorial_py', anonymous=False)

def create_object():
    scene = moveit_commander.PlanningSceneInterface(synchronous=True)
    scene.remove_world_object();
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.position.x = 0.5
    pose.pose.position.y = numpy.random.uniform(-0.1, 0.1)
    pose.pose.position.z = 0.16
    pose.pose.orientation.w = 1.0
    scene.add_box('object', pose, size=(0.05, 0.05, 0.2))

group = "panda_arm"
eef_frame = "panda_link8"

# Cartesian interpolation planner
cartesian = core.CartesianPath()

task = core.Task()

# start from current robot state
task.add(stages.CurrentState("current state"))

move = stages.MoveTo("to object", cartesian)
move.group = group
header = Header(frame_id = "object")
move.setGoal(PoseStamped(header=header, pose=Pose(position=Point(0,0,0.18),orientation=Quaternion(0.92388, -0.38268, 0, 0))))
task.add(move)

create_object()
if task.plan():
    task.publish(task.solutions[0])
rospy.spin()
