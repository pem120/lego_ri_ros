#!/usr/bin/env python3
"""
SerialInterface.py

A simple tty-based interface to allow bi-directional communication with the Lego Mindstorms RI/Lego Spike Prime
hub.

The interface operates at 115200 baud 8N1 on the port specified.
"""
from math import pi
from json import loads

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
from std_msgs.msg import String

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class LegoInterfaceNode(Node):
    def __init__(self):
        super().__init__("lego_spike_interface_node")
        print("init")
        self.logger = self.get_logger()
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

        self.command_queue = CommandList()
        self.timer_period = 1.0 / 50.0  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.run)

    def goal_pos_callback(self, joint_goal_state):
        """Handle incoming joint state commands"""
        self.logger.error("Received movement command:")
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
        """The main sensor-reading, command-sending loop.  The Lego Hub doesn't support threading, so this is a bit clunky"""

        datastr = str(self.read_line()).split("\n")
        try:
            if len(self.command_queue) > 0:
                print("trasmit")
                print(str(self.command_queue))
                self.command_queue.transmit(self)
            start_index = datastr[0].find("/{")
            if start_index != -1:
                datastr = datastr[0][start_index + 1 :]
            data = loads(datastr)
            # if not (data["imu"] or data["devices"] or data["temperature"]):
            #   self.logger.error(f"Invaid message from hub: {data}")
            #   continue
            if len(data["err"]) > 0:
                for e in data["err"]:
                    self.logger.error(f"Error in response from hub: {e}")
            self.send_ros_msgs(data)
        except Exception as err:
            self.logger.error(str(err))
        # rate.sleep()

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
        self.logger.error("Not implented by this class")

    def close(self):
        self.logger.error("Not implented by this class")

    def read_line(self):
        self.logger.error("Not implented by this class")

    def write_line(self, txt):
        self.logger.error("Not implented by this class")

    def write_byte(self, ch):
        self.logger.error("Not implented by this class")
