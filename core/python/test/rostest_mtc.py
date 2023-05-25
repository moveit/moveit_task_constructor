#! /usr/bin/env python

from __future__ import print_function
import unittest
import rostest
from moveit_commander.roscpp_initializer import roscpp_initialize
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header
import rospy


class Test(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    def test_MoveAndExecute(self):
        moveRel = stages.MoveRelative("moveRel", core.JointInterpolationPlanner())
        moveRel.group = self.PLANNING_GROUP
        moveRel.setDirection({"joint_1": 0.2, "joint_2": 0.4})

        moveTo = stages.MoveTo("moveTo", core.JointInterpolationPlanner())
        moveTo.group = self.PLANNING_GROUP
        moveTo.setGoal("all-zeros")

        task = core.Task()
        task.add(stages.CurrentState("current"), moveRel, moveTo)

        self.assertTrue(task.plan())
        self.assertEqual(len(task.solutions), 1)
        task.execute(task.solutions[0])

    def test_Merger(self):
        cartesian = core.CartesianPath()

        def createDisplacement(group, displacement):
            move = stages.MoveRelative("displace", cartesian)
            move.group = group
            move.ik_frame = PoseStamped(header=Header(frame_id="tool0"))
            dir = Vector3Stamped(header=Header(frame_id="base_link"), vector=Vector3(*displacement))
            move.setDirection(dir)
            move.restrictDirection(stages.MoveRelative.Direction.FORWARD)
            return move

        task = core.Task()
        task.add(stages.CurrentState("current"))
        merger = core.Merger("merger")
        merger.insert(createDisplacement(self.PLANNING_GROUP, [-0.2, 0, 0]))
        merger.insert(createDisplacement(self.PLANNING_GROUP, [0.2, 0, 0]))
        task.add(merger)

        task.enableIntrospection()
        task.init()
        self.assertFalse(task.plan())


if __name__ == "__main__":
    roscpp_initialize("test_mtc")
    rostest.rosrun("mtc", "base", Test)
