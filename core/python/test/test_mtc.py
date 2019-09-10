#! /usr/bin/env python
# -*- coding: utf-8 -*-

import unittest, sys
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, TwistStamped, Vector3Stamped
from moveit_msgs.msg import RobotState, Constraints, MotionPlanRequest
from moveit.task_constructor import core, stages


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
        # MotionPlanRequest is not registered as property type and should raise
        self.assertRaises(TypeError,self._check, "request", MotionPlanRequest())

    def test_assign_in_reference(self):
        planner = core.PipelinePlanner()
        props = planner.properties

        props["goal_joint_tolerance"] = 3.14
        self.assertEqual(props["goal_joint_tolerance"], 3.14)
        self.assertEqual(planner.goal_joint_tolerance, 3.14)

        planner.goal_joint_tolerance = 2.71
        self.assertEqual(props["goal_joint_tolerance"], 2.71)

        props["planner"] = "planner"
        self.assertEqual(props["planner"], "planner")
        self.assertEqual(planner.planner, "planner")

        props["double"] = 3.14
        a = props
        props["double"] = 2.71
        self.assertEqual(a["double"], 2.71)

        planner.planner = "other"
        self.assertEqual(props["planner"], "other")
        self.assertEqual(planner.planner, "other")

        del planner
        # TODO: Why can we still access props? planner should be destroyed
        self.assertEqual(props["goal_joint_tolerance"], 2.71)
        self.assertEqual(props["planner"], "other")

    def test_iter(self):
        # assign values so we can iterate over them
        self.props["double"] = 3.14
        self.props["bool"] = True
        first = [p for p in self.props]
        self.assertEqual(len(first), 2)
        second = [(name, value) for (name, value) in self.props]
        self.assertEqual(first, second)

    def test_update(self):
        self.props["double"] = 3.14
        self.props.update({"double": 2.72, "bool": True})
        self.props.update({})
        self.assertEqual(self.props["double"], 2.72)
        self.assertEqual(self.props["bool"], True)

    def test_expose(self):
        self.props["double"] = 3.14

        other = core.PropertyMap()
        self.props.exposeTo(other, "double")
        self.assertEqual(other["double"], self.props["double"])

        self.props.exposeTo(other, "double", "float")
        self.assertEqual(other["float"], self.props["double"])


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
        self._check_invalid_args(stage, name, type(value))

    def _check_assign(self, stage, name, value):
        setattr(stage, name, value)
        self.assertEqual(getattr(stage, name), value)

    def _check_invalid_args(self, stage, name, target_type):
        """Check some basic types to raise an ArgumentError when assigned"""
        for value in [None, 1, 1.0, "string", [], {}, set()]:
            try:
                target_type(value)
                continue  # ignore values that are implicitly convertible to target_type
            except:
                pass

            try:
                setattr(stage, name, value)
            except TypeError as e:
                pass
            except:
                self.fail("Assigning {} did raise wrong exception: {}".format(value, sys.exc_info()[0]))
            else:
                self.fail("Assigning {} did not raise an exception, result: {}".format(value, getattr(stage, name)))

    def test_CurrentState(self):
        stage = stages.CurrentState("current")

    def test_FixedState(self):
        stage = stages.FixedState("fixed")

    def test_ComputeIK(self):
        generator_stage = stages.GeneratePose("generator")
        stage = stages.ComputeIK("IK", generator_stage)

        self._check(stage, "timeout", 0.5)
        self._check(stage, "eef", "eef")
        self._check(stage, "group", "group")
        self._check(stage, "default_pose", "default_pose")
        self._check(stage, "max_ik_solutions", 1)
        self.assertRaises(OverflowError, self._check_assign, stage, "max_ik_solutions", -1)
        self._check(stage, "ignore_collisisons", False)
        self._check(stage, "ignore_collisisons", True)
        self._check(stage, "ik_frame", PoseStamped())
        self._check(stage, "target_pose", PoseStamped())
        self._check(stage, "forwarded_properties", ["name1", "name2", "name3"])

    def test_MoveTo(self):
        stage = stages.MoveTo("move", self.planner)

        self._check(stage, "group", "group")
        self._check(stage, "ik_frame", PoseStamped())
        stage.setGoal(PoseStamped())
        # TODO: fails
        # stage.setGoal(PointStamped())
        stage.setGoal(RobotState())
        self._check(stage, "path_constraints", Constraints())

    def test_MoveRelative(self):
        stage = stages.MoveRelative("move", self.planner)

        self._check(stage, "group", "group")
        self._check(stage, "ik_frame", PoseStamped())
        self._check(stage, "min_distance", 0.5)
        self._check(stage, "max_distance", 0.25)
        self._check(stage, "path_constraints", Constraints())
        stage.setDirection(TwistStamped())
        stage.setDirection(Vector3Stamped())
        stage.setDirection({'joint': 0.1})

    def test_Connect(self):
        planner = core.PipelinePlanner()
        planner2 = core.PipelinePlanner()
        stage = stages.Connect("connect", [("planner", planner), ("planner2", planner2)])

    def test_FixCollisionObjects(self):
        stage = stages.FixCollisionObjects("collision")

        self._check(stage, "max_penetration", 0.5)

    def test_GenerateGraspPose(self):
        stage = stages.GenerateGraspPose("generate_grasp_pose")

        self._check(stage, "eef", "eef")
        self._check(stage, "pregrasp", "pregrasp")
        self._check(stage, "object", "object")
        self._check(stage, "angle_delta", 0.5)

    def test_GeneratePose(self):
        stage = stages.GeneratePose("generate_pose")

        self._check(stage, "pose", PoseStamped())

    def test_Pick(self):
        generator_stage = stages.GeneratePose("generator")
        stage = stages.Pick(generator_stage, "pick")

        self._check(stage, "object", "object")
        self._check(stage, "eef", "eef")
        self._check(stage, "eef_frame", "eef_frame")
        self._check(stage, "eef_group", "eef_group")
        self._check(stage, "eef_parent_group", "eef_parent_group")

    def test_Place(self):
        generator_stage = stages.GeneratePose("generator")
        stage = stages.Place(generator_stage, "place")

        self._check(stage, "object", "object")
        self._check(stage, "eef", "eef")
        self._check(stage, "eef_frame", "eef_frame")
        self._check(stage, "eef_group", "eef_group")
        self._check(stage, "eef_parent_group", "eef_parent_group")

    def test_SimpleGrasp(self):
        stage = stages.SimpleGrasp(stages.GenerateGraspPose("grasp"))

        self._check(stage, "eef", "eef")
        self._check(stage, "object", "object")

    def test_SimpleUnGrasp(self):
        stage = stages.SimpleUnGrasp(stages.GenerateGraspPose("ungrasp"))

        self._check(stage, "eef", "eef")
        self._check(stage, "object", "object")

    def test_PropertyMaps(self):
        for name in dir(stages):
            if name.startswith("__") or name.endswith("__"):
                continue

            stage = getattr(stages, name)
            try:
                props = stage().properties
            except:
                continue

            try:
                for p in props:
                    pass
            except Exception as ex:
                print("error in class {}: {}".format(stage, ex))
                raise


class TestTask(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestTask, self).__init__(*args, **kwargs)

    def test(self):
        task = core.Task()
        self.assertEqual(task.id, "")
        task = core.Task("foo", core.SerialContainer())
        self.assertEqual(task.id, "foo")
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
