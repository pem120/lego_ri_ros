#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""

import argparse
import json
from math import pi
import os

import serial
from time import sleep
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
        self.logger = self.get_logger()

        # Publishers
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

        # Subscribers - fixed subscription creation
        self.goal_pos_sub = self.create_subscription(
            JointState,
            "cmd/goal_position",
            self.goal_pos_callback,
            qos_profile_sensor_data,
        )
        self.light_pattern_sub = self.create_subscription(
            LightPattern,
            "cmd/lights",
            self.cmd_lights_callback,
            qos_profile_sensor_data,
        )

        # Create timer with appropriate period (20Hz)
        self.run_timer = self.create_timer(0.05, self.run)
        self.command_queue = CommandList()

    def goal_pos_callback(self, joint_goal_state):
        """Handle incoming joint state commands"""
        self.logger.info("Received movement command:")
        param = {
            "name": list(joint_goal_state.name),
            "position": list(joint_goal_state.position),
            "velocity": list(joint_goal_state.velocity),
            "effort": list(joint_goal_state.effort),
        }
        # Log detailed movement parameters
        for i, name in enumerate(param["name"]):
            self.logger.info(f"Motor {name}:")
            self.logger.info(f"  Position: {param['position'][i]:.4f}")
            self.logger.info(f"  Velocity: {param['velocity'][i]:.4f}")
            self.logger.info(f"  Effort: {param['effort'][i]:.4f}")

        self.command_queue.append(CommandList.ACTION_MOTORS, param)

    def cmd_lights_callback(self, pattern):
        self.command_queue.append(CommandList.ACTION_LIGHTS, pattern.pattern)

    def run(self):
        """Timer callback for reading sensor data and sending commands"""
        if not hasattr(self, "port") or not self.port.isOpen():
            self.logger.error("Serial port not open")
            return

        try:
            # Process any pending commands first
            if True:  # len(self.command_queue) > 0:
                self.command_queue.transmit(self)
            datastr = str(self.read_line())

            # Clean up string and find JSON
            datastr = datastr.replace("'", '"')
            start_idx = datastr.find("{")
            if start_idx == -1:
                return

            # Find complete JSON object
            brace_count = 0
            end_idx = -1
            for i in range(start_idx, len(datastr)):
                if datastr[i] == "{":
                    brace_count += 1
                elif datastr[i] == "}":
                    brace_count -= 1
                    if brace_count == 0:
                        end_idx = i + 1
                        break

            if end_idx == -1:
                return

            # Parse and process JSON
            json_str = datastr[start_idx:end_idx]
            try:
                data = json.loads(json_str)
                if "imu" in data:
                    self.publish_imu(data["imu"])
                if "temperature" in data:
                    self.publish_temperature(data["temperature"])
                if "devices" in data:
                    self.publish_device_data(data["devices"])
            except json.JSONDecodeError as err:
                self.logger.error(f"JSON decode error: {err}")

        except serial.SerialException as err:
            self.logger.error(f"Serial error: {err}")
            self.close()
        except Exception as err:
            self.logger.error(f"Unexpected error: {str(err)}")

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

    def publish_imu(self, imu_data):
        """Publish IMU data if available"""
        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = self.imu_frame_id
        imu_msg = Imu()
        imu_msg.header = hdr
        imu_msg.angular_velocity.x = imu_data["angular"]["x"] * pi / 180.0  # deg2rad
        imu_msg.angular_velocity.y = imu_data["angular"]["y"] * pi / 180.0  # deg2rad
        imu_msg.angular_velocity.z = imu_data["angular"]["z"] * pi / 180.0  # deg2rad
        imu_msg.linear_acceleration.x = imu_data["linear"]["x"]
        imu_msg.linear_acceleration.y = imu_data["linear"]["y"]
        imu_msg.linear_acceleration.z = imu_data["linear"]["z"]
        self.imu_pub.publish(imu_msg)

    def publish_temperature(self, temp_data):
        """Publish temperature data if available"""
        temperature_msg = Float32()
        temperature_msg.data = temp_data
        self.temperature_pub.publish(temperature_msg)

    def publish_device_data(self, devices):
        """Publish device data if available"""
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

        for device in devices:
            if device["type"] == "motor":
                motor_name = f"motor_{device['port']}_wheel_joint"
                position = device["data"]["position"]
                speed = device["data"]["speed"]

                # Log motor state
                # self.logger.info(f"Motor {motor_name} state:")
                # self.logger.info(f"  Current Position: {position:.4f}")
                # self.logger.info(f"  Current Speed: {speed:.4f}")

                js.name.append(motor_name)
                js.position.append(position)
                js.velocity.append(speed)
                js.effort.append(0.0)
            elif device["type"] == "light":
                clr.name.append(f"color_{device['port']}")
                c = Color()
                c.brightness = device["data"]["level"]
                c.r = device["data"]["rgb"]["r"]
                c.g = device["data"]["rgb"]["g"]
                c.b = device["data"]["rgb"]["b"]
                clr.data.append(c)
            elif device["type"] == "distance":
                h = Header()
                h.stamp = self.get_clock().now().to_msg()
                h.frame_id = f"distance_{device['port']}"
                pc = PointCloud()
                pc.header = h
                points = [Point32()]
                points[0].x = float("NaN") if device["data"] < 0 else device["data"]
                points[0].y = 0.0
                points[0].z = 0.0
                pc.points = points
                channels = [ChannelFloat32()]
                channels[0].name = "distance"
                channels[0].values = [points[0].x]
                pc.channels = channels
                dst.data.append(pc)

        # Only publish messages if they have data
        if js.name:
            self.joint_state_pub.publish(js)
        if clr.name:
            self.color_sensors_pub.publish(clr)
        if dst.data:
            self.distance_sensors_pub.publish(dst)

    def open(self):
        self.logger.error("Not implented by this class")

    def close(self):
        self.logger.error("Not implented by this class")

    def read_line(self):
        self.logger.error("Not implented by this class")

    def write_line(self, txt):
        """Writes a single line of text with newline terminator to the micropython interpreter"""
        try:
            if not isinstance(txt, str):
                txt = str(txt)
            data = (txt + "\n").encode("utf-8")
            if self.verbose:
                self.logger.info(f"Writing: {txt}")
            self.port.write(data)
        except Exception as err:
            self.logger.error(f"Write error: {err}")

    def write_byte(self, ch):
        """Writes a single character to the micropython interpreter"""
        try:
            if isinstance(ch, bytes):
                self.port.write(ch)
            else:
                self.port.write(bytes([ch]))
            if self.verbose:
                self.logger.info(f"Writing byte: {ch}")
        except Exception as err:
            self.logger.error(f"Write byte error: {err}")


class SerialInterfaceNode(LegoInterfaceNode):
    """Handles bidirectional communication over the USB virtual com port of the Lego Hub"""

    def __init__(self, port="com5", baud=115200, verbose=False):
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
            self.logger.debug("port is already open")
            return
        try:
            self.port.open()
        except serial.SerialException as err:
            self.logger.error(err)

    def close(self):
        if not self.port.isOpen():
            self.logger.debug("port is already closed")
            return
        try:
            self.port.close()
        except Exception as err:
            self.logger.error(err)

    def read_line(self):
        "Reads a single line of text from the micropython interpreter"
        l = self.port.readline()
        l = l.decode("utf-8").rstrip()
        if self.verbose:
            self.logger.info(l)
        return l

    def write_line(self, txt):
        "Writes a single line of text with newline terminator to the micropython interpreter"
        try:
            if not isinstance(txt, str):
                txt = str(txt)
            data = (txt + "\n").encode("utf-8")
            if self.verbose:
                self.logger.info(f"Writing: {txt}")
            self.port.write(data)
        except Exception as err:
            self.logger.error(f"Write error: {err}")

    def write_byte(self, ch):
        "Writes a single character to the micropython interpreter"
        try:
            if isinstance(ch, bytes):
                self.port.write(ch)
            else:
                self.port.write(bytes([ch]))
            if self.verbose:
                self.logger.info(f"Writing byte: {ch}")
        except Exception as err:
            self.logger.error(f"Write byte error: {err}")


def main():
    parser = argparse.ArgumentParser(
        "Serial interface for Lego Mindstorms and Lego Spike Prime hub"
    )
    parser.add_argument(
        "-p",
        "--port",
        metavar="TTY",
        type=str,
        default="com5",
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
        help="Serial port baud rate (default 115200)",
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
    # Create node
    node = SerialInterfaceNode(port=args.port, baud=args.baud, verbose=args.verbose)

    # Initialize serial connection
    node.get_logger().info(f"Opening port {args.port}...")

    # Define shutdown handler
    rclpy.spin(node)


if __name__ == "__main__":
    main()
    exit()
