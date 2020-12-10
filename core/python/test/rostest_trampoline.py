#! /usr/bin/env python

from __future__ import print_function
import unittest
import rostest
from moveit.python_tools import roscpp_init
from moveit.task_constructor import core, stages
from geometry_msgs.msg import Vector3Stamped, Vector3
from std_msgs.msg import Header

PLANNING_GROUP = "manipulator"


class PyGenerator(core.Generator):
    """ Implements a custom 'Generator' stage."""

    max_calls = 3

    def __init__(self, name="Generator"):
        core.Generator.__init__(self, name)
        self.reset()

    def reset(self):
        core.Generator.reset(self)
        self.num = self.max_calls

    def canCompute(self):
        return self.num > 0

    def compute(self):
        self.num = self.num - 1


class PyMonitoringGenerator(core.MonitoringGenerator):
    """ Implements a custom 'MonitoringGenerator' stage."""

    def __init__(self, name="MonitoringGenerator"):
        core.MonitoringGenerator.__init__(self, name)

    def onNewSolution(self, sol):
        pass


class PyMoveRelX(stages.MoveRelative):
    """ Implements a custom propagator stage."""

    def __init__(self, x, planner, name="Move ±x"):
        stages.MoveRelative.__init__(self, name, planner)
        self.group = PLANNING_GROUP
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

    def create(self, *stages):
        task = core.Task()
        task.enableIntrospection()
        for stage in stages:
            task.add(stage)
        return task

    def plan(self, task, expected_solutions=None, wait=False):
        task.plan()
        if expected_solutions is not None:
            assert len(task.solutions) == expected_solutions
        if wait:
            input("Waiting for any key (allows inspection in rviz)")

    def test_generator(self):
        task = self.create(PyGenerator())
        self.plan(task, expected_solutions=PyGenerator.max_calls)

    def test_monitoring_generator(self):
        task = self.create(
            stages.CurrentState("current"),
            stages.Connect(planners=[(PLANNING_GROUP, self.jointspace)]),
        )
        gen = PyMonitoringGenerator()
        gen.setMonitoredStage(task["current"])
        task.add(gen)
        self.plan(task)

    def test_propagator(self):
        task = self.create(
            PyMoveRelX(-0.2, self.cartesian),
            stages.CurrentState(),
            PyMoveRelX(+0.2, self.cartesian),
        )
        self.plan(task, expected_solutions=1)


if __name__ == "__main__":
    roscpp_init("test_mtc")
    rostest.rosrun("mtc", "trampoline", TestTrampolines)
