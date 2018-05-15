#! /usr/bin/env python
# -*- coding: utf-8 -*-

from moveit.task_constructor import core, stages
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
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


class TestModifyPlanningScene(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestModifyPlanningScene, self).__init__(*args, **kwargs)
        self.mps = stages.ModifyPlanningScene("mps")

    def test_attach_objects_invalid_args(self):
        for value in [None, 1, 1.5, {}]:
            self.assertRaises(TypeError, self.mps.attachObjects, value, "link")
            self.assertRaises(TypeError, self.mps.attachObjects, value, "link", True)
            self.assertRaises(TypeError, self.mps.attachObjects, value, "link", False)

    def test_attach_objects_valid_args(self):
        self.mps.attachObject("object", "link")
        self.mps.detachObject("object", "link")

        self.mps.attachObjects("object", "link")
        self.mps.detachObjects("object", "link")
        self.mps.attachObjects("object", "link", True)
        self.mps.attachObjects("object", "link", False)

        self.mps.attachObjects([], "link")
        self.mps.attachObjects(["object"], "link")
        self.mps.attachObjects(["object1", "object2", "object3"], "link")

    def test_allow_collisions(self):
        self.mps.allowCollisions("first", "second")
        self.mps.allowCollisions("first", "second", True)
        self.mps.allowCollisions("first", "second", False)

        self.mps.allowCollisions(["first"], ["second"])


class TestStages(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestStages, self).__init__(*args, **kwargs)
        self.planner = core.PipelinePlanner()

    def _check(self, stage, name, value):
        self._check_assign(stage, name, value)
        try:
            float(value)
            ignored_types = [int, float, long]
        except (TypeError, ValueError):
            ignored_types = [type(value)]
        self._check_invalid_args(stage, name, ignored_types)

    def _check_assign(self, stage, name, value):
        setattr(stage, name, value)
        self.assertEqual(getattr(stage, name), value)

    def _check_invalid_args(self, stage, name, ignored_types = None):
        """Check some basic types to raise an ArgumentError when assigned"""
        if ignored_types is None:
            ignored_types = []
        for value in [None, 1, 1.0, "string", [], {}, set()]:
            if type(value) in ignored_types:
                continue
            try:
                setattr(stage, name, value)
            except TypeError as e:
                pass
            else:
                self.fail("Assigning {} did not raise an exception.".format(value))

    def test_ComputeIK(self):
        stage = stages.GeneratePose("generator")
        compute_ik = stages.ComputeIK("IK", stage)

    def test_MoveTo(self):
        stage = stages.MoveTo("move", self.planner)

        self._check(stage, "pose", PoseStamped())

        # TODO missing tests for "pose", "point", "joint_pose" and "path_constraints" properties

    def test_moveRelative(self):
        stage = stages.MoveRelative("move", self.planner)

        self._check(stage, "timeout", 0.5)
        self._check(stage, "marker_ns", "marker_ns")
        self._check(stage, "group", "group")
        self._check(stage, "link", "link")
        self._check(stage, "min_distance", 0.5)
        self._check(stage, "max_distance", 0.25)

        self._check(stage, "joints", {})
        self._check_assign(stage, "joints", {"half": 0.5, "quarter": 0.25, "zero": 0})

        # TODO missing tests for "twist", "direction" and "path_constraints" properties

        stage.joints = {"one": 1}
        self.assertEqual(stage.joints["one"], 1)

        # TODO dictionary key/val-pair is not deleted. should not be equal and raise a key error here
        del stage.joints["one"]
        # self.assertNotEqual(stage.joints["one"], 1)

        # TODO clearing the dict does not work either. should be equal
        stage.joints.clear()
        # self.assertEqual(stage.joints, {})


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
