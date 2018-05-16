#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import Pose
import unittest


class TestPropertyMap(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestPropertyMap, self).__init__(*args, **kwargs)
        self.props = core.PropertyMap()

    def _check(self, name, value):
        self.props[name] = value
        self.assertEqual(self.props[name], value)

    def test_assign(self):
        self._check("double", 3.14)
        self._check("long", 42)
        self._check("long", 13)
        self._check("bool", True)
        self._check("bool", False)
        self._check("string", "anything")
        self._check("pose", Pose())

    def test_assign_in_reference(self):
        planner = core.PipelinePlanner()
        props = planner.properties

        props["timeout"] = 3.14
        self.assertEqual(props["timeout"], 3.14)
        self.assertEqual(planner.timeout, 3.14)

        planner.timeout = 2.71
        self.assertEqual(props["timeout"], 2.71)

        props["group"] = "mygroup"
        self.assertEqual(props["group"], "mygroup")
        self.assertEqual(planner.group, "mygroup")

        planner.group = "other"
        self.assertEqual(props["group"], "other")
        self.assertEqual(planner.group, "other")

        del planner
        # TODO: Why can we still access props? planner should be destroyed
        self.assertEqual(props["timeout"], 2.71)
        self.assertEqual(props["group"], "other")


class TestTask(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestTask, self).__init__(*args, **kwargs)

    def test(self):
        task = core.Task("task")
        self.assertEqual(task.id, "task")

        current = stages.CurrentState("current")
        self.assertEqual(current.name, "current")
        current.timeout = 1.23
        self.assertEqual(current.timeout, 1.23)

        task.add(current)

        # ownership of current was passed to task
        with self.assertRaises(TypeError):
            current.name

        task.add(stages.Connect("connect", []))
        task.add(stages.FixedState())


if __name__ == '__main__':
    unittest.main()
