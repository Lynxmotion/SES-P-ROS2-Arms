#!/usr/bin/env -S ros2 launch
"""Example of planning with MoveIt2 within RViz2 and simulating motions using Ignition Gazebo ROS2 control plugin"""

from os import path
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    world = LaunchConfiguration("world")
    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
    gripper = LaunchConfiguration("gripper")
    finger = LaunchConfiguration("finger")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_verbosity = LaunchConfiguration("sim_verbosity")
    log_level = LaunchConfiguration("log_level")

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
            launch_arguments=[("gz_args", [world, " -r -v ", sim_verbosity])],
        ),
        # Launch move_group of MoveIt 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("pro_arm_moveit"),
                        "launch",
                        "move_arm.launch.py",
                    ]
                )
            ),
            launch_arguments=[
                ("dof", dof),
                ("size", size),
                ("gripper", gripper),
                ("finger", finger),
                ("ros2_control_plugin", "sim"),
                ("collision", "true"),
                ("rviz_config", rviz_config),
                ("use_sim_time", use_sim_time),
                ("log_level", log_level),
            ],
        ),
    ]

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
        # ros_gz_bridge (clock -> ROS 2)
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                "--ros-args",
                "--log-level",
                log_level,
            ],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + launch_descriptions + nodes)


def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
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
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("pro_arm_moveit"),
                "rviz",
                "moveit.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
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
