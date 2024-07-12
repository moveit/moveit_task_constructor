#! /usr/bin/env python3

import unittest
import rclcpp
from moveit.task_constructor import core, stages
from geometry_msgs.msg import PoseStamped


def setUpModule():
    rclcpp.init()


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
        self.make_box = self.psi._PlanningSceneInterface__make_box
        # insert a box to collide with
        self.psi.add_box("box", make_pose(0.8, 0.25, 1.25), [0.2, 0.2, 0.2])
        self.node = rclcpp.Node("test_mtc")
        self.task = task = core.Task()
        self.task.loadRobotModel(self.node)
        task.add(stages.CurrentState("current"))

    def insertMove(self, position=-1):
        # colliding motion
        move = stages.MoveRelative("move", core.JointInterpolationPlanner())
        move.group = self.PLANNING_GROUP
        move.setDirection({"joint_1": 0.3})
        self.task.insert(move, position)

    def test_collision(self):
        self.insertMove()
        self.assertFalse(self.task.plan())

    def test_allow_collision_list(self):
        mps = stages.ModifyPlanningScene("allow specific collisions for box")
        mps.allowCollisions("box", ["link_4", "link_5", "link_6"], True)
        self.task.add(mps)
        self.insertMove()
        self.assertTrue(self.task.plan())

    def test_allow_collision_all(self):
        # insert an extra collision object that is unknown to ACM
        self.psi.add_box("block", make_pose(0.8, 0.55, 1.25), [0.2, 0.2, 0.2])
        # attach box to end effector
        self.psi.attach_box("link_6", "box")
        self.insertMove()
        self.assertFalse(self.task.plan())

        # allow all collisions for attached "box" object
        mps = stages.ModifyPlanningScene("allow all collisions for box")
        mps.allowCollisions("box", True)
        self.task.insert(mps, 1)
        self.assertTrue(self.task.plan())
        # restore original state
        self.psi.remove_attached_object("link_6", "box")
        self.psi.remove_world_object("block")

    def test_fw_add_object(self):
        mps = stages.ModifyPlanningScene("addObject(block)")
        mps.addObject(self.make_box("block", make_pose(0.8, 0.55, 1.25), [0.2, 0.2, 0.2]))
        self.task.add(mps)
        self.insertMove()
        self.assertFalse(self.task.plan())

    def test_fw_remove_object(self):
        mps = stages.ModifyPlanningScene("removeObject(box)")
        mps.removeObject("box")
        self.task.insert(mps, 1)
        self.assertTrue(self.task.plan())
        s = self.task.solutions[0].toMsg()
        self.assertEqual(s.sub_trajectory[1].scene_diff.world.collision_objects[0].id, "box")

    def test_bw_add_object(self):
        # add object to move_group's planning scene
        self.psi.add_box("block", make_pose(0.8, 0.55, 1.25), [0.2, 0.2, 0.2])

        # backward operation will actually remove the object
        mps = stages.ModifyPlanningScene("addObject(block) backwards")
        mps.addObject(self.make_box("block", make_pose(0.8, 0.55, 1.25), [0.2, 0.2, 0.2]))
        self.task.insert(mps, 0)
        self.insertMove(0)
        self.assertTrue(self.task.plan())

        self.psi.remove_world_object("block")  # restore original state

        s = self.task.solutions[0].toMsg()

        # block shouldn't be in start scene
        objects = [o.id for o in s.start_scene.world.collision_objects]
        self.assertTrue(objects == ["box"])

        # only addObject(block) should add it
        objects = [o.id for o in s.sub_trajectory[1].scene_diff.world.collision_objects]
        self.assertTrue(objects == ["block", "box"])

    def test_bw_remove_object(self):
        mps = stages.ModifyPlanningScene("removeObject(box) backwards")
        mps.removeObject("box")
        self.task.insert(mps, 0)
        self.insertMove(0)
        self.assertFalse(self.task.plan())


if __name__ == "__main__":
    unittest.main()
