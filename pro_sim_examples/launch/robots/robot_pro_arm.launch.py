#!/usr/bin/env -S ros2 launch
"""Launch script for spawning Pro 5DoF/6DoF Arm into Ignition Gazebo world"""

from typing import List

from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
    gripper = LaunchConfiguration("gripper")
    finger = LaunchConfiguration("finger")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    model = PythonExpression(["'pro_arm_", dof, "dof_", size, "_", gripper, "_", finger, ".sdf'"])

    # List of nodes to be launched
    nodes = [
        # ros_gz_sim_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Version
        DeclareLaunchArgument(
            "dof",
            default_value='6',
            choices=['5','6'],
            description="Parameter to select dof version."
        ),
        DeclareLaunchArgument(
            "size",
            default_value='550',
            choices=['550','900'],
            description="Parameter to select size version."
        ),
        DeclareLaunchArgument(
            "gripper",
            default_value='none',
            choices=['none','pge_5040','cge_1010'],
            description="Parameter to select gripper model."
        ),
        DeclareLaunchArgument(
            "finger",
            default_value='40',
            choices=['20','40','60','80'],
            description="Parameter to select finger separation model."
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
