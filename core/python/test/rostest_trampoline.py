#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest
import rclcpp
from moveit.task_constructor import core, stages
from moveit.core.planning_scene import PlanningScene
from geometry_msgs.msg import Vector3Stamped, Vector3, PoseStamped
from std_msgs.msg import Header

PLANNING_GROUP = "manipulator"


def setUpModule():
    rclcpp.init()


def pybind11_versions():
    try:
        keys = __builtins__.keys()  # for use with pytest
    except AttributeError:
        keys = __builtins__.__dict__.keys()  # use from cmdline
    return [k for k in keys if k.startswith("__pybind11_internals_v")]


incompatible_pybind11_msg = "MoveIt and MTC use incompatible pybind11 versions: " + "\n- ".join(
    pybind11_versions()
)


class PyGenerator(core.Generator):
    """Implements a custom 'Generator' stage."""

    max_calls = 3

    def __init__(self, name="Generator"):
        core.Generator.__init__(self, name)
        self.reset()

    def init(self, robot_model):
        self.ps = PlanningScene(robot_model)

    def reset(self):
        core.Generator.reset(self)
        self.num = self.max_calls

    def canCompute(self):
        return self.num > 0

    def compute(self):
        self.num = self.num - 1
        self.spawn(core.InterfaceState(self.ps), self.num)


class PyMonitoringGenerator(core.MonitoringGenerator):
    """Implements a custom 'MonitoringGenerator' stage."""

    solution_multiplier = 2

    def __init__(self, name="MonitoringGenerator"):
        core.MonitoringGenerator.__init__(self, name)
        self.reset()

    def reset(self):
        core.MonitoringGenerator.reset(self)
        self.upstream_solutions = list()

    def onNewSolution(self, sol):
        self.upstream_solutions.append(sol)

    def canCompute(self):
        return bool(self.upstream_solutions)

    def compute(self):
        scene = self.upstream_solutions.pop(0).end.scene
        for i in range(self.solution_multiplier):
            self.spawn(core.InterfaceState(scene), i)


class PyMoveRelX(stages.MoveRelative):
    """Implements a custom propagator stage."""

    def __init__(self, x, planner, name="Move ±x"):
        stages.MoveRelative.__init__(self, name, planner)
        self.group = PLANNING_GROUP
        self.ik_frame = PoseStamped(header=Header(frame_id="tool0"))
        self.setDirection(
            Vector3Stamped(header=Header(frame_id="base_link"), vector=Vector3(x, 0, 0))
        )

    def computeForward(self, from_state):
        return stages.MoveRelative.computeForward(self, from_state)

    def computeBackward(self, to_state):
        return stages.MoveRelative.computeBackward(self, to_state)


class TestTrampolines(unittest.TestCase):
    def setUp(self):
        self.cartesian = core.CartesianPath()
        self.jointspace = core.JointInterpolationPlanner()
        self.node = rclcpp.Node("test_mtc")

    def create(self, *stages):
        task = core.Task()
        task.loadRobotModel(self.node)
        task.enableIntrospection()
        for stage in stages:
            task.add(stage)
        return task

    def plan(self, task, expected_solutions=None, wait=False):
        task.plan()
        if expected_solutions is not None:
            self.assertEqual(len(task.solutions), expected_solutions)
        if wait:
            input("Waiting for any key (allows inspection in rviz)")

    @unittest.skipIf(len(pybind11_versions()) > 1, incompatible_pybind11_msg)
    def test_generator(self):
        task = self.create(PyGenerator())
        self.plan(task, expected_solutions=PyGenerator.max_calls)

    @unittest.skipIf(len(pybind11_versions()) > 1, incompatible_pybind11_msg)
    def test_monitoring_generator(self):
        task = self.create(
            stages.CurrentState("current"),
            stages.Connect(planners=[(PLANNING_GROUP, self.jointspace)]),
            PyMonitoringGenerator("generator"),
        )
        task["generator"].setMonitoredStage(task["current"])
        self.plan(task, expected_solutions=PyMonitoringGenerator.solution_multiplier)

    def test_propagator(self):
        task = self.create(
            PyMoveRelX(-0.2, self.cartesian),
            stages.CurrentState(),
            PyMoveRelX(+0.2, self.cartesian),
        )
        self.plan(task, expected_solutions=1)


if __name__ == "__main__":
    unittest.main()
