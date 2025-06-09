#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""

import argparse
from math import pi
import os
import serial

from lego_spike_interface.command import CommandList

from lego_spike_msgs.msg import Color
from lego_spike_msgs.msg import ColorSensors
from lego_spike_msgs.msg import DistanceSensors
from lego_spike_msgs.msg import LightPattern
from geometry_msgs.msg import Point32
from sensor_msgs.msg import ChannelFloat32
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from std_msgs.msg import Float32

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


ctrl_c = b"\x03"
ctrl_e = b"\x05"
ctrl_d = b"\x04"


class LegoInterfaceNode(Node):
    def __init__(self):
        super().__init__("lego_spike_interface_node")

        # TODO parameterize all these
        self.imu_frame_id = "lego_hub_imu_link"

        self.imu_pub = self.create_publisher(Imu, "imu/data", qos_profile_sensor_data)
        self.temperature_pub = self.create_publisher(
            Float32, "temperature", qos_profile_sensor_data
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "joint_states", qos_profile_sensor_data
        )
        self.color_sensors_pub = self.create_publisher(
            ColorSensors, "colors", qos_profile_sensor_data
        )
        self.distance_sensors_pub = self.create_publisher(
            DistanceSensors, "distance", qos_profile_sensor_data
        )

        self.goal_pos_sub = self.create_subscription(
            JointState,
            "cmd/goal_position",
            self.goal_pos_callback,
            qos_profile_sensor_data,
        )
        self.light_pattern_sub = self.create_subscriber(
            LightPattern,
            "cmd/lights",
            self.cmd_lights_callback,
            qos_profile_sensor_data,
        )

        self.command_queue = CommandList()

    def goal_pos_callback(self, joint_goal_state):
        param = {
            "name": list(joint_goal_state.name),
            "position": list(joint_goal_state.position),
            "velocity": list(joint_goal_state.velocity),
            "effort": list(joint_goal_state.effort),
        }
        self.command_queue.append(CommandList.ACTION_MOTORS, param)

    def cmd_lights_callback(self, pattern):
        self.command_queue.append(CommandList.ACTION_LIGHTS, pattern.pattern)

    def send_main(self, path=None):
        "Sends main.py to the Lego Hub so it can communicate bidirectionally"

        # to avoid serial corruption in paste-mode send the data one line at a time at 50 Hz
        rate = self.create_rate(50)

        # cancel whatever's running first!
        for i in range(3):
            self.write_byte(ctrl_c)
            rate.sleep()

        # skip past all the cruft during startup & the initial interpreter lines
        self.get_logger().info("Reading past boot messages...")
        l = self.read_line()
        while not l.startswith(">>>"):
            l = self.read_line()
            self.get_logger().debug(l)
        self.get_logger().info("Reading past boot interpreter prompt...")
        while l.startswith(">>>"):
            l = self.read_line()
            self.get_logger().debug(l)

        self.get_logger().info("Sending Lego Hub main at {0}".format(path))
        file_in = open(path, "r")
        lines = file_in.readlines()
        file_in.close()
        self.get_logger().info("Entering paste mode...")
        self.write_byte(ctrl_e)
        l = self.read_line()
        while len(l.strip()) > 0:
            l = self.read_line()
            rate.sleep()
        for l in lines:
            # don't send empty lines
            check_l = l.rstrip()
            if len(check_l) > 0:
                self.get_logger().debug(check_l)
                self.write_line(l)
                self.read_line()
                rate.sleep()
        self.read_line()

        self.get_logger().info("Exiting paste mode...")
        self.write_byte(ctrl_d)
        self.read_line()

        self.get_logger().info("Lego Hub main sent!")

    def run(self):
        "The main sensor-reading, command-sending loop.  The Lego Hub doesn't support threading, so this is a bit clunky"

        # our main sensor-reading loop runs at 10Hz
        # TODO: can we go faster?
        rate = self.create_rate(50)

        while rclpy.ok():
            datastr = self.read_line()
            try:
                data = eval(datastr)
                if len(data["err"]) > 0:
                    for e in data["err"]:
                        self.get_logger().error(
                            "Error in response from hub: {0}".format(e)
                        )

                self.send_ros_msgs(data)

                if len(self.command_queue) > 0:
                    self.command_queue.transmit(self)

            except Exception as err:
                self.get_logger().error(err)
            rate.sleep()

        # cancel when we're done
        self.port.write(ctrl_c)

    def send_ros_msgs(self, data):
        "Publish the ROS messages"
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.imu_frame_id
        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.angular_velocity.x = data["imu"]["angular"]["x"] * pi / 180.0  # deg2rad
        imu_msg.angular_velocity.y = data["imu"]["angular"]["y"] * pi / 180.0  # deg2rad
        imu_msg.angular_velocity.z = data["imu"]["angular"]["z"] * pi / 180.0  # deg2rad
        imu_msg.linear_acceleration.x = data["imu"]["linear"]["x"]
        imu_msg.linear_acceleration.y = data["imu"]["linear"]["y"]
        imu_msg.linear_acceleration.z = data["imu"]["linear"]["z"]

        temperature_msg = Float32()
        temperature_msg.data = data["temperature"]

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header = hdr
        js.name = []
        js.position = []
        js.effort = []
        js.velocity = []
        clr = ColorSensors()
        clr.name = []
        dst = DistanceSensors()
        dst.data = []

        for device in data["devices"]:
            if device["type"] == "motor":
                js.name.append("motor_{0}_wheel_joint".format(device["port"]))
                js.position.append(device["data"]["position"])
                js.velocity.append(device["data"]["speed"])
                js.effort.append(0.0)  # TODO?
            elif device["type"] == "light":
                clr.name.append("color_{0}".format(device["port"]))
                c = Color()
                c.brightness = device["data"]["level"]
                c.r = device["data"]["rgb"]["r"]
                c.g = device["data"]["rgb"]["g"]
                c.b = device["data"]["rgb"]["b"]
                clr.data.append(c)
            elif device["type"] == "distance":
                h = Header()
                h.stamp = self.get_clock().now().to_msg()
                h.frame_id = "distance_{0}".format(device["port"])
                pc = PointCloud()
                pc.header = h
                points = [Point32()]
                if device["data"] < 0:
                    points[0].x = float("NaN")
                else:
                    points[0].x = device["data"]
                points[0].y = 0.0
                points[0].z = 0.0
                pc.points = points
                channels = [ChannelFloat32()]
                channels[0].name = "distance"
                channels[0].values = [points[0].x]
                pc.channels = channels
                dst.data.append(pc)

        self.imu_pub.publish(imu_msg)
        self.temperature_pub.publish(temperature_msg)
        self.joint_state_pub.publish(js)
        self.color_sensors_pub.publish(clr)
        self.distance_sensors_pub.publish(dst)

    def open(self):
        self.get_logger().error("Not implented by this class")

    def close(self):
        self.get_logger().error("Not implented by this class")

    def read_line(self):
        self.get_logger().error("Not implented by this class")

    def write_line(self, txt):
        self.get_logger().error("Not implented by this class")

    def write_byte(self, ch):
        self.get_logger().error("Not implented by this class")


class SerialInterfaceNode(LegoInterfaceNode):
    """Handles bidirectional communication over the USB virtual com port of the Lego Hub"""

    def __init__(self, port="/dev/lego", baud=115200, verbose=False):
        super().__init__()
        self.verbose = verbose

        self.port = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1,
        )

    def open(self):
        if self.port.isOpen():
            self.get_logger().debug("port is already open")
            return
        try:
            self.port.open()
        except serial.SerialException as err:
            self.get_logger().error(err)

    def close(self):
        if not self.port.isOpen():
            self.get_logger().debug("port is already closed")
            return
        try:
            self.port.close()
        except Exception as err:
            self.get_logger().error(err)

    def read_line(self):
        "Reads a single line of text from the micropython interpreter"
        l = self.port.readline()
        l = l.decode("utf-8").rstrip()
        if self.verbose:
            self.get_logger().info(l)
        return l

    def write_line(self, txt):
        "Writes a single line of text with newline terminator to the micropython interpreter"
        data = bytes(txt, encoding="utf-8")
        self.port.write(data)

    def write_byte(self, ch):
        "Writes a single character to the micropython interpreter"
        self.port.write(ch)

    def send_main(self):
        path = os.path.join(
            get_package_share_directory("lego_spike_interface"), "mindstorms", "main.py"
        )
        super().send_main(path=path)


def main():
    parser = argparse.ArgumentParser(
        "Serial interface for Lego Mindstorms and Lego Spike Prime hub"
    )
    parser.add_argument(
        "-p",
        "--port",
        metavar="TTY",
        type=str,
        default="/dev/lego",
        dest="port",
        help="Serial port to open (default /dev/lego)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        metavar="INT",
        type=int,
        default=115200,
        dest="baud",
        help="Serial port to open (default /dev/lego)",
    )
    parser.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        dest="verbose",
        help="Show all serial I/O from the hub",
    )
    args, unk_args = parser.parse_known_args()

    rclpy.init()
    node = SerialInterfaceNode(port=args.port, baud=args.baud, verbose=args.verbose)
    node.open()
    node.send_main()
    rclpy.spin(node)
    node.close()
    rclpy.shutdown()
