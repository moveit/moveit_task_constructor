import unittest

import launch_testing
import pytest
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from moveit_configs_utils import MoveItConfigsBuilder


@pytest.mark.launch_test
def generate_test_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_fanuc").to_moveit_configs()

    test_exec = Node(
        executable=[LaunchConfiguration("exe")],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    return (
        LaunchDescription(
            [
                DeclareLaunchArgument(name="exe"),
                test_exec,
                KeepAliveProc(),
                ReadyToTest(),
            ]
        ),
        {"test_exec": test_exec},
    )


class TestTerminatingProcessStops(unittest.TestCase):
    def test_gtest_run_complete(self, proc_info, test_exec):
        proc_info.assertWaitForShutdown(process=test_exec, timeout=4000.0)


@launch_testing.post_shutdown_test()
class TaskModelTestAfterShutdown(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
