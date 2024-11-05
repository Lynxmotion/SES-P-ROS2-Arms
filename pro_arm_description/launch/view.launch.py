from os import path
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:

    # Declare all launch arguments
    declared_arguments = generate_declared_arguments()

    # Get substitution for all arguments
    description_package = LaunchConfiguration("description_package")
    description_filepath = LaunchConfiguration("description_filepath")
    name = LaunchConfiguration("name")
    prefix = LaunchConfiguration("prefix")
    dof = LaunchConfiguration("dof")
    size = LaunchConfiguration("size")
    gripper = LaunchConfiguration("gripper")
    finger = LaunchConfiguration("finger")
    # collision = LaunchConfiguration("collision")
    ros2_control = LaunchConfiguration("ros2_control")
    ros2_control_plugin = LaunchConfiguration("ros2_control_plugin")
    # gazebo_preserve_fixed_joint = LaunchConfiguration("gazebo_preserve_fixed_joint")
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    log_level = LaunchConfiguration("log_level")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), description_filepath]
            ),
            " ",
            "name:=",
            name,
            " ",
            "prefix:=",
            prefix,
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
            finger,
            " ",
            "ros2_control:=",
            ros2_control,
            " ",
            "ros2_control_plugin:=",
            ros2_control_plugin,
            " "
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    nodes = [
            # robot_state_publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="log",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[robot_description, {"use_sim_time": use_sim_time}],
            ),
            # rviz2
            Node(
                package="rviz2",
                executable="rviz2",
                output="log",
                arguments=[
                    "--display-config",
                    rviz_config,
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            # joint_state_publisher_gui
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="log",
                arguments=["--ros-args", "--log-level", log_level],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    
    return LaunchDescription(declared_arguments + nodes)

def generate_declared_arguments() -> List[DeclareLaunchArgument]:
    """
    Generate list of all launch arguments that are declared for this launch script.
    """

    return [
        # Location of xacro/URDF to visualise
        DeclareLaunchArgument(
            "description_package",
            default_value="pro_arm_description",
            description="Custom package with robot description.",
        ),
        DeclareLaunchArgument(
            "description_filepath",
            default_value="urdf/pro_arm.urdf.xacro",
            description="Path to xacro or URDF description of the robot, relative to share of `description_package`.",
        ),
        # Naming of the robot
        DeclareLaunchArgument(
            "name",
            default_value="pro_arm",
            description="Name of the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for all robot entities. If modified, then joint names in the configuration of controllers must also be updated.",
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
        # ROS 2 control
        DeclareLaunchArgument(
            "ros2_control",
            default_value="true",
            description="Flag to enable ros2 controllers for manipulator.",
        ),
        DeclareLaunchArgument(
            "ros2_control_plugin",
            default_value="sim",
            description="The ros2_control plugin that should be loaded for the manipulator ('fake', 'ign', 'real' or custom).",
        ),
        # Miscellaneous
        DeclareLaunchArgument(
            "rviz_config",
            default_value=path.join(
                get_package_share_directory("pro_arm_description"),
                "rviz",
                "view.rviz",
            ),
            description="Path to configuration for RViz2.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="If true, use simulated clock.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="The level of logging that is applied to all ROS 2 nodes launched by this script.",
        ),
    ]