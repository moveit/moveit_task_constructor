#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, TwistStamped
import rospy
import time

rospy.init_node("PickPlacePython")

# Specify robot parameters
arm = "panda_arm"
eef = "hand"

# Specify object parameters
object_name = "grasp_object"
object_radius = 0.02

# Start with a clear planning scene
psi = PlanningSceneInterface()
time.sleep(0.2)
psi.remove_world_object()
time.sleep(0.2)

# Grasp object properties
objectRadius = 1.0
objectPose = PoseStamped()
objectPose.header.frame_id = "world"
objectPose.pose.orientation.x = 1.0
objectPose.pose.position.x = 0.30702
objectPose.pose.position.y = 0.0
objectPose.pose.position.z = 0.285

# Add the grasp object to the planning scene
time.sleep(0.2)
psi.add_sphere(object_name, objectPose, object_radius)
time.sleep(0.2)

# Create a task
task = core.Task("PandaPickPipelineExample")
task.enableIntrospection()

# Start with the current state
current = stages.CurrentState("current")
task.add(current)

# Create a planner instance that is used to connect
# the current state to the grasp approach pose
pipelinePlanner = core.PipelinePlanner()
pipelinePlanner.planner = "RRTConnectkConfigDefault"
planners = [(arm, pipelinePlanner)]

# Connect the two stages
connect = stages.Connect("connect1", planners)
connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
task.add(connect)

# The grasp generator spawns a set of possible grasp
# poses around the object
grasp_generator = stages.GenerateGraspPose("Generate Grasp Pose")
grasp_generator.angle_delta = 0.2
grasp_generator.pregrasp = "open"
grasp_generator.grasp = "close"
grasp_generator.setMonitoredStage(task["current"])

# Make sure the object is not getting placed
# in-collision with the base frame of the panda's gripper
ikFrame = PoseStamped()
ikFrame.header.frame_id = "panda_hand"
ikFrame.pose.position.z = 0.09

# Generate a simple grasp for every grasp generator pose.
# This stage wraps the inverse kinematic computation
simpleGrasp = stages.SimpleGrasp(grasp_generator, "Simple Grasp")
simpleGrasp.setIKFrame(ikFrame)

# Pick pipeline wrapper
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

# Add the pick stage to the task's stage hierarchy
task.add(pick)

# Connect the pick stage with the now following
# place stage
connect = stages.Connect("connect2", planners)
connect.properties.configureInitFrom(core.Stage.PropertyInitializerSource.PARENT)
task.add(connect)

# Define the pose that the object should obtain
# when it is placed
placePose = PoseStamped()
placePose.header.frame_id = "world"
placePose.pose = objectPose.pose
placePose.pose.position.y += 0.2  # modify place pose relative to pick pose

# The grasp generator samples possible grasp
# frames around the destination pose.
grasp_generator = stages.GeneratePlacePose("Generate Place Pose")
grasp_generator.setMonitoredStage(task["Pick"])
grasp_generator.object = object_name
grasp_generator.pose = placePose
grasp_generator.properties.configureInitFrom(
    core.Stage.PropertyInitializerSource.PARENT, ["ik_frame"]
)

# Generate a grasp for every sampled pose by again wrapping
# inverse kinematics computation
simpleUnGrasp = stages.SimpleUnGrasp(grasp_generator, "Simple Grasp")
simpleUnGrasp.setIKFrame(ikFrame)
simpleUnGrasp.grasp = "close"
simpleUnGrasp.pregrasp = "open"

# Place wrapper stage
place = stages.Place(simpleUnGrasp, "Place")
place.eef = eef
place.object = object_name
place.eef_frame = "panda_link8"

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

if task.plan():
    task.publish(task.solutions[0])
time.sleep(1)

# Prevent the program from exiting,
# giving you the opportunity to inspect the solutions
# visually via the solution topic in rviz.
rospy.spin()
