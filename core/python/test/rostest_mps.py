#! /usr/bin/env python

from __future__ import print_function
import unittest
import rostest
from moveit_commander.roscpp_initializer import roscpp_initialize
from moveit_commander import PlanningSceneInterface
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped


def make_pose(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = 1.0
    return pose


class TestModifyPlanningScene(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    def setUp(self):
        super(TestModifyPlanningScene, self).setUp()
        self.psi = PlanningSceneInterface(synchronous=True)
        # insert a box to collide with
        self.psi.add_box("box", make_pose(0.8, 0.25, 1.25), [0.2, 0.2, 0.2])

        # colliding motion
        move = stages.MoveRelative("move", core.JointInterpolationPlanner())
        move.group = self.PLANNING_GROUP
        move.setDirection({"joint_1": 0.3})

        self.task = task = core.Task()
        task.add(stages.CurrentState("current"), move)

    def test_collision(self):
        self.assertFalse(self.task.plan())

    def test_allow_collision_list(self):
        mps = stages.ModifyPlanningScene("mps")
        mps.allowCollisions("box", ["link_4", "link_5", "link_6"], True)
        self.task.insert(mps, 1)
        self.assertTrue(self.task.plan())

    def test_allow_collision_all(self):
        # insert an extra collision object that is unknown to ACM
        self.psi.add_box("block", make_pose(0.8, 0.55, 1.25), [0.2, 0.2, 0.2])
        # attach box to end effector
        self.psi.attach_box("link_6", "box")
        mps = stages.ModifyPlanningScene("mps")
        self.assertFalse(self.task.plan())

        # allow all collisions for attached "box" object
        mps.allowCollisions("box", True)
        self.task.insert(mps, 1)
        self.assertTrue(self.task.plan())
        # restore original state
        self.psi.remove_attached_object("link_6", "box")
        self.psi.remove_world_object("block")


if __name__ == "__main__":
    roscpp_initialize("test_mtc")
    rostest.rosrun("mtc", "mps", TestModifyPlanningScene)
