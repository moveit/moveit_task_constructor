import unittest
import os

from ament_index_python.packages import get_package_share_directory
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
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .planning_pipelines(pipelines=["ompl"])
        .robot_description(file_path="config/panda.urdf.xacro")
        .to_moveit_configs()
    )

    # Load  ExecuteTaskSolutionCapability so we can test executing solutions
    move_group_capabilities = {"capabilities": "move_group/ExecuteTaskSolutionCapability"}

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
    )

    # Publish joint states to TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller",
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

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
                static_tf,
                robot_state_publisher,
                move_group_node,
                ros2_control_node,
                controller_spawner,
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
