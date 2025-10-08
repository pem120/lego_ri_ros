#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages
    pkg_lego_spike_description = FindPackageShare("lego_spike_description")

    # Launch Configurations
    robot_description_command = LaunchConfiguration("robot_description_command")
    robot_config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    arg_robot_config = DeclareLaunchArgument(
        "config",
        choices=["basic", "simple_arm"],
        default_value="basic",
        description="The Lego model configuration",
    )
    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        choices=["true", "false"],
        default_value="false",
    )

    robot_urdf = PathJoinSubstitution(
        [
            pkg_lego_spike_description,
            "urdf",
            robot_config,
        ]
    )

    # Get URDF via xacro
    arg_robot_description_command = DeclareLaunchArgument(
        "robot_description_command",
        default_value=[
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_urdf,
            ".urdf.xacro",
            " ",
            "is_sim:=",
            use_sim_time,
            " ",
        ],
    )

    robot_description_content = ParameterValue(
        Command(robot_description_command), value_type=str
    )

    group_action_state_publishers = GroupAction(
        [
            # Robot State Publisher
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description_content,
                        "use_sim_time": use_sim_time,
                    }
                ],
                remappings=[
                    ("/tf", "tf"),
                    ("/tf_static", "tf_static"),
                    ("joint_states", "platform/joint_states"),
                ],
            ),
        ]
    )

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_robot_config)
    ld.add_action(arg_robot_description_command)
    # Nodes
    ld.add_action(group_action_state_publishers)
    return ld
