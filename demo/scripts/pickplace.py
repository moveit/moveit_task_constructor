#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rclcpp
from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import Constraints, OrientationConstraint
import math, time

rclcpp.init()
node = rclcpp.Node("mtc_tutorial")

# [pickAndPlaceTut1]
# Specify robot parameters
arm = "panda_arm"
eef = "hand"
# [pickAndPlaceTut1]

# [pickAndPlaceTut2]
# Specify object parameters
object_name = "object"
object_radius = 0.02

# Start with a clear planning scene
psi = PlanningSceneInterface(synchronous=True)
psi.remove_world_object()

# [initCollisionObject]
# Grasp object properties
objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.orientation.w = 1.0
objectPose.pose.position.x = 0.30702
objectPose.pose.position.y = 0.0
objectPose.pose.position.z = 0.285
# [initCollisionObject]

# Add the grasp object to the planning scene
psi.add_box(object_name, objectPose, size=[0.1, 0.05, 0.03])
# [pickAndPlaceTut2]

# [pickAndPlaceTut3]
# Create a task
task = core.Task()
task.name = "pick + place"
# [pickAndPlaceTut3]

# [pickAndPlaceTut4]
# Start with the current state
task.add(stages.CurrentState("current"))

# [initAndConfigConnect]
# Create a planner instance that is used to connect
# the current state to the grasp approach pose
pipeline = core.PipelinePlanner(node, "ompl", "RRTConnectkConfigDefault")
planners = [(arm, pipeline)]

# Connect the two stages
task.add(stages.Connect("connect", planners))
# [initAndConfigConnect]
# [pickAndPlaceTut4]

# [pickAndPlaceTut5]
# [initAndConfigGenerateGraspPose]
# The grasp generator spawns a set of possible grasp poses around the object
grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
grasp_generator.angle_delta = math.pi / 2
grasp_generator.pregrasp = "open"
grasp_generator.grasp = "close"
grasp_generator.setMonitoredStage(task["current"])  # Generate solutions for all initial states
# [initAndConfigGenerateGraspPose]
# [pickAndPlaceTut5]

# [pickAndPlaceTut6]
# [initAndConfigSimpleGrasp]
# SimpleGrasp container encapsulates IK calculation of arm pose as well as finger closing
simpleGrasp = stages.SimpleGrasp(grasp_generator, "Grasp")
# Set frame for IK calculation in the center between the fingers
ik_frame = PoseStamped()
ik_frame.header.frame_id = "panda_hand"
ik_frame.pose.position.z = 0.1034  # tcp between fingers
ik_frame.pose.orientation.x = 1.0  # grasp from top
simpleGrasp.setIKFrame(ik_frame)
# [initAndConfigSimpleGrasp]
# [pickAndPlaceTut6]

# [pickAndPlaceTut7]
# [initAndConfigPick]
# Pick container comprises approaching, grasping (using SimpleGrasp stage), and lifting of object
pick = stages.Pick(simpleGrasp, "Pick")
pick.eef = eef
pick.object = object_name

# Twist to approach the object
approach = TwistStamped()
approach.header.frame_id = "world"
approach.twist.linear.z = -1.0
pick.setApproachMotion(approach, 0.03, 0.1)

# Twist to lift the object
lift = TwistStamped()
lift.header.frame_id = "panda_hand"
lift.twist.linear.z = -1.0
pick.setLiftMotion(lift, 0.03, 0.1)
# [pickAndPlaceTut7]

# [pickAndPlaceTut8]
# Add the pick stage to the task's stage hierarchy
task.add(pick)
# [initAndConfigPick]
# [pickAndPlaceTut8]

# Define orientation constraint to keep the object upright
oc = OrientationConstraint()
oc.parameterization = oc.ROTATION_VECTOR
oc.header.frame_id = "world"
oc.link_name = "object"
oc.orientation.w = 1
oc.absolute_x_axis_tolerance = 0.1
oc.absolute_y_axis_tolerance = 0.1
oc.absolute_z_axis_tolerance = math.pi
oc.weight = 1.0

constraints = Constraints()
constraints.name = "object:upright"
constraints.orientation_constraints.append(oc)

# [pickAndPlaceTut9]
# Connect the Pick stage with the following Place stage
con = stages.Connect("connect", planners)
con.path_constraints = constraints
task.add(con)
# [pickAndPlaceTut9]

# [pickAndPlaceTut10]
# [initAndConfigGeneratePlacePose]
# Define the pose that the object should have after placing
placePose = objectPose
placePose.pose.position.x = -0.2
placePose.pose.position.y = -0.6
placePose.pose.position.z = 0.0

# Generate Cartesian place poses for the object
place_generator = stages.GeneratePlacePose("Generate Place Pose")
place_generator.setMonitoredStage(task["Pick"])
place_generator.object = object_name
place_generator.pose = placePose
# [initAndConfigGeneratePlacePose]
# [pickAndPlaceTut10]

# [initAndConfigSimpleUnGrasp]
# The SimpleUnGrasp container encapsulates releasing the object at the given Cartesian pose
# [pickAndPlaceTut11]
simpleUnGrasp = stages.SimpleUnGrasp(place_generator, "UnGrasp")
# [pickAndPlaceTut11]

# [pickAndPlaceTut12]
# [initAndConfigPlace]
# Place container comprises placing, ungrasping, and retracting
place = stages.Place(simpleUnGrasp, "Place")
place.eef = eef
place.object = object_name
place.eef_frame = "panda_link8"
# [initAndConfigSimpleUnGrasp]

# Twist to retract from the object
retract = TwistStamped()
retract.header.frame_id = "world"
retract.twist.linear.z = 1.0
place.setRetractMotion(retract, 0.03, 0.1)

# Twist to place the object
placeMotion = TwistStamped()
placeMotion.header.frame_id = "panda_hand"
placeMotion.twist.linear.z = 1.0
place.setPlaceMotion(placeMotion, 0.03, 0.1)

# Add the place pipeline to the task's hierarchy
task.add(place)
# [initAndConfigPlace]
# [pickAndPlaceTut12]

# [pickAndPlaceTut13]
if task.plan():
    task.publish(task.solutions[0])

# avoid ClassLoader warning
del pipeline
del planners
# [pickAndPlaceTut13]

# Prevent the program from exiting, giving you the opportunity to inspect solutions in rviz
time.sleep(3600)
