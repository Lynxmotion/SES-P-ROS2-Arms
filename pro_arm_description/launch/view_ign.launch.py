#!/usr/bin/env -S ros2 launch
"""Visualisation of SDF model for PRO 5DoF/6DoF Arm in Ignition Gazebo."""

from os import path
from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    world = LaunchConfiguration("world")
    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
    gripper = LaunchConfiguration("gripper")
    finger = LaunchConfiguration("finger")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_verbosity = LaunchConfiguration("sim_verbosity")
    log_level = LaunchConfiguration("log_level")

    model = PythonExpression(["'pro_arm_", dof, "dof_", size, "_", gripper, "_", finger, ".sdf'"])

    # URDF
    _robot_description_xml = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            model,
            " ",
            "dof:=",
            dof,
            " ",
            "size:=",
            size,
            " ",
            "gripper:=",
            gripper,
            " ",
            "finger:=",
            finger
        ]
    )
    robot_description = {"robot_description": _robot_description_xml}

    # List of included launch descriptions
    launch_descriptions = [
        # Launch Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ),
            launch_arguments=[("gz_args", [world, " -v ", sim_verbosity])],
        ),
    ]

    # List of nodes to be launched
    nodes = [
        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="log",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                robot_description,
                {
                    "publish_frequency": 30.0,
                    "frame_prefix": "",
                    "use_sim_time": use_sim_time,
                },
            ],
        ),
        # ros_gz_sim_create
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-file", model, "--ros-args", "--log-level", log_level],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Locations of robot resources
        DeclareLaunchArgument(
            "description_package",
            default_value="pro_arm_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value=path.join("urdf", "pro_arm.urdf.xacro"),
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # World and model for Ignition Gazebo
        DeclareLaunchArgument(
            "world",
            default_value="default.sdf",
            description="Name or filepath of world to load.",
        ),
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
            "sim_verbosity",
            default_value="3",
            description="Verbosity level for Ignition Gazebo (0~4).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]
