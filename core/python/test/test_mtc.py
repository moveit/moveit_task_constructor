#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import pytest
import unittest
import rclcpp
from geometry_msgs.msg import Pose, PoseStamped, PointStamped, TwistStamped, Vector3Stamped
from moveit_msgs.msg import RobotState, Constraints, MotionPlanRequest
from moveit.task_constructor import core, stages


def setUpModule():
    rclcpp.init()


def tearDownModule():
    rclcpp.shutdown()


# When py_binding_tools and MTC are compiled with different pybind11 versions,
# the corresponding classes are not interoperable.
def check_pybind11_incompatibility():
    rclcpp.init()
    node = rclcpp.Node("dummy")
    try:
        core.PipelinePlanner(node)
    except TypeError:
        return True
    finally:
        rclcpp.shutdown()
    return False


incompatible_pybind11 = check_pybind11_incompatibility()
incompatible_pybind11_msg = "MoveIt and MTC use incompatible pybind11 versions"


class TestPropertyMap(unittest.TestCase):

    def setUp(self):
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
        self._check("pose", PoseStamped())
        # MotionPlanRequest is not registered as property type and should raise
        self.assertRaises(TypeError, self._check, "request", MotionPlanRequest())

    @unittest.skipIf(incompatible_pybind11, incompatible_pybind11_msg)
    def test_assign_in_reference(self):
        node = rclcpp.Node("test_mtc_props")
        planner = core.PipelinePlanner(node)
        props = planner.properties

        props["goal_joint_tolerance"] = 3.14
        self.assertEqual(props["goal_joint_tolerance"], 3.14)
        self.assertEqual(planner.goal_joint_tolerance, 3.14)

        planner.goal_joint_tolerance = 2.71
        self.assertEqual(props["goal_joint_tolerance"], 2.71)

        props["planner"] = "planner"
        self.assertEqual(props["planner"], "planner")

        props["double"] = 3.14
        a = props
        props["double"] = 2.71
        self.assertEqual(a["double"], 2.71)

        del planner
        # We can still access props, because actual destruction of planner is delayed
        self.assertEqual(props["goal_joint_tolerance"], 2.71)

    def test_iter(self):
        # assign values so we can iterate over them
        self.props["double"] = 3.14
        self.props["bool"] = True
        keys = [v for v in self.props]
        self.assertEqual(len(keys), 2)
        items = [(k, v) for (k, v) in self.props.items()]
        self.assertEqual(keys, [k for (k, v) in items])

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
    def setUp(self):
        self.mps = stages.ModifyPlanningScene("mps")

    def test_attach_objects_invalid_args(self):
        for value in [None, 1, 1.5, {}]:
            self.assertRaises(RuntimeError, self.mps.attachObjects, value, "link")
            self.assertRaises(RuntimeError, self.mps.attachObjects, value, "link", True)
            self.assertRaises(RuntimeError, self.mps.attachObjects, value, "link", False)

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
    @unittest.skipIf(incompatible_pybind11, incompatible_pybind11_msg)
    def setUp(self):
        self.node = rclcpp.Node("test_mtc_stages")
        self.planner = core.PipelinePlanner(self.node)

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
            except TypeError:
                pass
            except:
                msg = "Assigning {} did raise wrong exception: {}"
                self.fail(msg.format(value, sys.exc_info()[0]))
            else:
                if value == "string" and target_type is PoseStamped:
                    continue  # string is convertible to PoseStamped
                msg = "Assigning {} did not raise an exception, result: {}"
                self.fail(msg.format(value, getattr(stage, name)))

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
        self.assertRaises(TypeError, self._check_assign, stage, "max_ik_solutions", -1)
        self._check(stage, "ignore_collisions", False)
        self._check(stage, "ignore_collisions", True)
        self._check(stage, "ik_frame", PoseStamped())
        self._check(stage, "target_pose", PoseStamped())
        self._check(stage, "forwarded_properties", ["name1", "name2", "name3"])
        stage.forwarded_properties = "name"
        self.assertRaises(TypeError, self._check_assign, stage, "forwarded_properties", [1, 2])

    def test_MoveTo(self):
        stage = stages.MoveTo("move", self.planner)

        self._check(stage, "group", "group")
        self._check(stage, "ik_frame", PoseStamped())
        self._check(stage, "path_constraints", Constraints())
        stage.setGoal(PoseStamped())
        stage.setGoal(PointStamped())
        stage.setGoal(RobotState())
        stage.setGoal("named pose")
        stage.setGoal(dict(joint1=1.0, joint2=2.0))
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
        stage.setDirection({"joint": 0.1})

    def test_Connect(self):
        stage = stages.Connect("connect", [("group1", self.planner), ("group2", self.planner)])

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

    def test_CostTerm(self):
        stage = stages.CurrentState()
        weights = {"joint_{}".format(i + 1): 1.0 for i in range(6)}
        costs = core.PathLength(weights)
        stage.setCostTerm(costs)


class BaseTestCases:
    class ContainerTest(unittest.TestCase):
        def __init__(self, ContainerType, *args, **kwargs):
            super(BaseTestCases.ContainerTest, self).__init__(*args, **kwargs)
            self.ContainerType = ContainerType
            self.container = container = ContainerType()
            container.add(stages.CurrentState("1"))
            container.add(stages.CurrentState("2"))
            container.add(stages.CurrentState("3"))

        def test_move(self):
            container = self.ContainerType()
            stage = stages.CurrentState()
            container.add(stage)
            with self.assertRaises(ValueError):
                stage.name

        def test_access_by_name(self):
            with self.assertRaises(IndexError):
                self.container["unknown"]

            child = self.container["2"]
            self.assertEqual(child.name, "2")

        def test_access_by_iterator(self):
            self.assertEqual([child.name for child in self.container], ["1", "2", "3"])

        def test_access_by_index(self):
            self.assertEqual(self.container[0].name, "1")
            self.assertEqual(self.container[1].name, "2")
            self.assertEqual(self.container[-1].name, "3")
            self.assertEqual(self.container[-2].name, "2")
            with self.assertRaises(IndexError):
                self.container[3]
            with self.assertRaises(IndexError):
                self.container[-4]


class TestSerial(BaseTestCases.ContainerTest):
    def __init__(self, *args, **kwargs):
        super(TestSerial, self).__init__(core.SerialContainer, *args, **kwargs)


class TestTask(BaseTestCases.ContainerTest):
    def __init__(self, *args, **kwargs):
        super(TestTask, self).__init__(core.Task, *args, **kwargs)

    def test(self):
        task = core.Task()
        current = stages.CurrentState("current")
        self.assertEqual(current.name, "current")
        current.timeout = 1.23
        self.assertEqual(current.timeout, 1.23)

        task.add(current)

        # ownership of current was passed to task
        with self.assertRaises(ValueError):
            current.name

        task.add(stages.Connect("connect", []))
        task.add(stages.FixedState())


if __name__ == "__main__":
    unittest.main()
