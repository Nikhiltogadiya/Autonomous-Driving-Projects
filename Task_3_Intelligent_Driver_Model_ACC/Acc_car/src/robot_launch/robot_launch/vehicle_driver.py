#!/usr/bin/env python

from turtle import rt
import rclpy
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Float32
from webots_interfaces.msg import RadarTarget
from pprint import pprint, pformat

import robot_test

class Driver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        self.vehicle_topic = properties["vehicle"]

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('webots_vehicle_node')
        self.__node.create_subscription(AckermannDrive,
                                        self.vehicle_topic + '/cmd_ackermann',
                                        self.__cmd_ackermann_callback, 1)
        self.__node.create_subscription(Float32,
                                        self.vehicle_topic + '/cmd_steering',
                                        self.__cmd_steering_callback, 1)
        self.__node.create_subscription(Float32,
                                        self.vehicle_topic + '/cmd_speed',
                                        self.__cmd_speed_callback, 1)

        self.__node.declare_parameter("testdefinition", "")
        self.__node.declare_parameter("testsuite", "")
        self.__node.declare_parameter("testcase", "")

        self.__testdefinition = ""
        self.__testcase_id = ""
        self.__testsuite_id = ""
        self.__testcase = None
        
        self.__radar = self.__robot.getDevice("sms")
        if self.__radar is not None:
            self.__radar_publisher = self.__node.create_publisher(
                RadarTarget, self.vehicle_topic + '/radar_targets', 1)
            self.__radar.enable(1)

    def __cmd_ackermann_callback(self, message):
        self.__robot.setCruisingSpeed(message.speed)
        self.__robot.setSteeringAngle(message.steering_angle)

    def __cmd_steering_callback(self, message):
        self.__robot.setSteeringAngle(message.data)

    def __cmd_speed_callback(self, message):
        self.__robot.setCruisingSpeed(message.data)

    def step(self):
        
        if self.__testcase is None:
            self.__testdefinition = self.__node.get_parameter(
                'testdefinition').get_parameter_value().string_value
            self.__testcase_id = self.__node.get_parameter(
                'testcase').get_parameter_value().string_value
            self.__testsuite_id = self.__node.get_parameter(
                'testsuite').get_parameter_value().string_value
            if self.__testdefinition != "" and self.__testsuite_id != "" and self.__testcase_id != "":
                self.__node.get_logger().info(f"got test definition, enable verification: {self.__testdefinition}.{self.__testsuite_id}.{self.__testcase_id}")
                self.__testcase = robot_test.Testcase(filename=self.__testdefinition, testsuite_id=self.__testsuite_id, robot=self.__robot, testcase_id=self.__testcase_id)
        else:
            self.__testcase.execute()

        if self.__radar is not None:
            for t in self.__radar.getTargets():
                rt = RadarTarget()
                rt.header.stamp = self.__node.get_clock().now().to_msg()
                rt.distance = t.distance
                rt.receiver_power = t.receiver_power
                rt.speed = t.speed
                rt.azimuth = t.azimuth
                self.__radar_publisher.publish(rt)

        rclpy.spin_once(self.__node, timeout_sec=0)
