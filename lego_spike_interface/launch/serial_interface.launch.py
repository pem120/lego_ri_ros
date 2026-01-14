from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        "port", default_value="/dev/lego", description="Serial port"
    )
    baudrate_arg = DeclareLaunchArgument(
        "baudrate", default_value="115200", description="Baud rate"
    )
    debug_arg = DeclareLaunchArgument(
        "debug", default_value="false", description="Enable debug (true/false)"
    )

    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    debug = LaunchConfiguration("debug")

    node_with_debug = Node(
        package="lego_spike_interface",
        executable="serial_interface",
        name="lego_serial_interface",
        arguments=["-p", port, "-b", baudrate, "-v"],
        condition=IfCondition(debug),
    )

    node_without_debug = Node(
        package="lego_spike_interface",
        executable="serial_interface",
        name="lego_serial_interface",
        arguments=["-p", port, "-b", baudrate],
        condition=UnlessCondition(debug),
    )

    return LaunchDescription(
        [
            port_arg,
            baudrate_arg,
            debug_arg,
            node_with_debug,
            node_without_debug,
        ]
    )
