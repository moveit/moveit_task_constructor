import unittest
import os

from ament_index_python.packages import get_package_share_directory
import launch_testing
import pytest
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.util import KeepAliveProc
from moveit_configs_utils import MoveItConfigsBuilder


@pytest.mark.launch_test
def generate_test_description():
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    demo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("moveit_task_constructor_demo"), "launch/demo.launch.py"
            )
        ),
        launch_arguments={"rviz": "false"}.items(),
    )

    # increase the timeout for the kinematics solver to fix flaky IK for asan
    k = moveit_config.robot_description_kinematics["robot_description_kinematics"]
    k["panda_arm"]["kinematics_solver_timeout"] = 5.0

    test_exec = Node(
        executable=[
            LaunchConfiguration("test_binary"),
        ],
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
                demo_bringup,
                DeclareLaunchArgument(
                    name="test_binary",
                    description="Test executable",
                ),
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
