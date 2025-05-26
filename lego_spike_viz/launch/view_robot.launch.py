from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    arg_robot_config = DeclareLaunchArgument(
        'config',
        choices=['basic', 'simple_arm'],
        default_value='basic',
    )

    arg_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    arg_rviz_config = DeclareLaunchArgument(
        name='rviz_config',
        default_value='lego.rviz',
    )

    # Launch Configurations
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get Package Paths
    pkg_lego_spike_viz = FindPackageShare('lego_spike_viz')

    config_rviz = PathJoinSubstitution(
        [pkg_lego_spike_viz, 'rviz', rviz_config]
    )

    group_view_model = GroupAction([
        Node(package='rviz2',
             executable='rviz2',
             name='rviz2',
             arguments=['-d', config_rviz],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[
               ('/tf', 'tf'),
               ('/tf_static', 'tf_static')
             ],
             output='screen'),
    ])

    ld = LaunchDescription()
    # Args
    ld.add_action(arg_robot_config)
    ld.add_action(arg_rviz_config)
    ld.add_action(arg_use_sim_time)
    # Nodes
    ld.add_action(group_view_model)

    return ld
