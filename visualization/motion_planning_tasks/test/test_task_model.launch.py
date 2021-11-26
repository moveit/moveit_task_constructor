import unittest

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import launch_testing
from launch_testing.asserts import assertExitCodes
from launch_testing.util import KeepAliveProc
from launch_testing.actions import ReadyToTest, GTest

import pytest


@pytest.mark.launch_test
def generate_test_description():
    test_task_model = GTest(
        path=[
            PathJoinSubstitution(
                [
                    LaunchConfiguration("test_binary_dir"),
                    "moveit_task_constructor_visualization-test-task_model",
                ]
            )
        ],
        output="screen",
    )
    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(
                    name="test_binary_dir",
                    description="Binary directory of package containing test executables",
                ),
                test_task_model,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {"test_task_model": test_task_model},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, test_task_model):
        proc_info.assertWaitForShutdown(process=test_task_model, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TaskModelTestAfterShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
