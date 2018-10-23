#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import rostest
from moveit.python_tools import roscpp_init
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import rospy


class Test(unittest.TestCase):
    PLANNING_GROUP = "manipulator"

    @classmethod
    def setUpClass(self):
        pass

    @classmethod
    def tearDown(self):
        pass

    def test_MoveRelative(self):
        task = core.Task()
        task.add(stages.CurrentState("current"))
        move = stages.MoveRelative("move", core.PipelinePlanner())
        move.group = self.PLANNING_GROUP
        move.setDirection({"joint_1" : 0.2, "joint_2" : 0.4})
        task.add(move)

        task.enableIntrospection()
        task.init()
        task.plan()

        self.assertEqual(len(task.solutions), 1)
        for s in task.solutions:
            print s
        s = task.solutions[0]
        task.execute(s)


if __name__ == '__main__':
    roscpp_init("test_mtc")
    rostest.rosrun("", "", Test)
