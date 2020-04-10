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

group = 'panda_arm'
eef = 'panda_hand'
eef_frame = "panda_link8"

sampling_planner = core.JointInterpolationPlanner()
cartesian_planner = core.CartesianPath()

task = core.Task()

task.properties.update({'group': group, 'eef': eef, 'hand': eef, 'hand_grasping_frame': eef, 'ik_frame': eef_frame})

currstate = stages.CurrentState('current state')
#task.add(currstate)  # Adding it to the task results in error for argument types in setMonitoredStage in GenerateGraspPose

open_hand = stages.MoveTo("open hand", sampling_planner)
open_hand.group = eef
open_hand.setGoal('open')
task.add(open_hand)

connect = stages.Connect('move to pick', [(group, sampling_planner)])
connect.timeout = 5
connect.properties.configureInitFrom(core.PARENT)
task.add(connect)

grasp = core.SerialContainer('pick object')
task.properties.exposeTo(grasp.properties, ['eef', 'hand', 'group', 'ik_frame'])
grasp.properties.configureInitFrom(core.PARENT, ['eef', 'hand', 'group', 'ik_frame'])

approach_object = stages.MoveRelative("approach_object", cartesian_planner)
approach_object.properties.update({'marker_ns': 'approach_object', 'link': eef_frame})
approach_object.properties.configureInitFrom(core.PARENT, ['group'])
approach_object.min_distance = 0.01
approach_object.max_distance = 0.1
print(approach_object.properties.__getitem__('group'))   # Why is this None? how to get properties from within SerialContainer?
approach_object.setDirection(TwistStamped(header=Header(frame_id = eef_frame), twist=Twist(linear=Vector3(0,0,0.1))))
grasp.insert(approach_object)

generatepose = stages.GenerateGraspPose('generate grasp pose')
generatepose.properties.configureInitFrom(core.PARENT)
generatepose.properties.update({'marker_ns': 'grasp_pose'})
generatepose.pregrasp = 'open'
generatepose.object = 'base'
generatepose.angle_delta = numpy.pi/12
generatepose.setMonitoredStage(currstate)

# To be continued
task.add(grasp)

if task.plan():
    task.publish(task.solutions[0])
rospy.spin()
