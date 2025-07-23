import os
from re import S
import cv2
import numpy as np
import math
from pprint import pprint, pformat

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from std_msgs.msg import Float32
from webots_interfaces.msg import RadarTarget
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.parameter import Parameter

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

def intelligentDrivingModel(desiredSpeed, currentSpeed, deltaSpeed, currentGap,
                            maxAcceleration, accelerationExponent, headwayTime,
                            comfortableDeceleration, safetyDist):
    # https://en.wikipedia.org/wiki/Intelligent_driver_model
    accValue = maxAcceleration * \
        (1.0 - math.pow(currentSpeed / desiredSpeed, accelerationExponent))
    param_1 = headwayTime * currentSpeed
    param_2 = (currentSpeed * deltaSpeed) / (
        2.0 * (math.pow(maxAcceleration * comfortableDeceleration, 0.5)))
    gap = max(safetyDist, param_1) + param_2

    if gap < 0.0:  # gap less than 0 - implies target faster than ego
        gap = 0.0

    brakeValue = maxAcceleration * math.pow(gap / currentGap, 2)
    return accValue - brakeValue


class ACCAgent(Node):

    def __init__(self):
        super().__init__('acc_agent')

        self.declare_parameter("vehicle", "")
        self.vehicle_topic = self.get_parameter(
            'vehicle').get_parameter_value().string_value
        self.declare_parameter("max_speed", 75.0)
        self.max_speed = self.get_parameter(
            'max_speed').get_parameter_value().double_value
        self.declare_parameter("max_acceleration", 2.0)
        self.max_acceleration = self.get_parameter(
            'max_acceleration').get_parameter_value().double_value
        self.declare_parameter("acceleration_exponent", 4.0)
        self.acceleration_exponent = self.get_parameter(
            'acceleration_exponent').get_parameter_value().double_value
        self.declare_parameter("headway_time", 1.8)
        self.headway_time = self.get_parameter(
            'headway_time').get_parameter_value().double_value
        self.declare_parameter("comfortable_deceleration", 2.0)
        self.comfortable_deceleration = self.get_parameter(
            'comfortable_deceleration').get_parameter_value().double_value
        self.declare_parameter("safety_distance", 5.0)
        self.safety_distance = self.get_parameter(
            'safety_distance').get_parameter_value().double_value

        self.set_parameters(
            [Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # ROS interface
        self.__speed_publisher = self.create_publisher(
            Float32, self.vehicle_topic + '/cmd_speed', 1)
        self.create_subscription(RadarTarget,
                                 self.vehicle_topic + '/radar_targets',
                                 self.__on_radar, qos_profile_sensor_data)
        self.create_subscription(Float32, self.vehicle_topic + '/gps/speed',
                                 self.__on_gps_speed, qos_profile_sensor_data)

        self.radar_distance = -1
        self.last_radar_dist_update = None

        self.gps_speed = -1
        self.last_gps_speed_update = None

        self.current_value = self.max_speed * 0.5

        self.timer = self.create_timer(0.1, self.__timer_callback)

    def __on_radar(self, message):
        if message.distance > 2:  # ghost in first curve
            self.radar_distance = message.distance
            self.radar_speed = message.speed
            self.last_radar_dist_update = self.get_clock().now()

    def __on_gps_speed(self, message):
        if message.data < 500:  # sometimes there is a very big value as first package
            self.gps_speed = message.data
            self.last_gps_speed_update = self.get_clock().now()

    def __timer_callback(self):

        if self.get_clock().now() - Time(
                nanoseconds=0,
                clock_type=self.get_clock().clock_type) < Duration(seconds=5):
            return
        s = Float32()
        s.data = self.current_value
        acc = 0

        # calculate speed value here
        current_time = self.get_clock().now()
        radar_valid = self.last_radar_dist_update is not None and (current_time - self.last_radar_dist_update) < Duration(seconds=1)
        gps_valid = self.last_gps_speed_update is not None and (current_time - self.last_gps_speed_update) < Duration(seconds=1)
        # take care of old, therefore, invalid data
        if radar_valid and gps_valid and self.max_speed > 0:
            delta_speed = self.gps_speed - (self.gps_speed + self.radar_speed)
            acc = intelligentDrivingModel(
                desiredSpeed=self.max_speed,
                currentSpeed=self.gps_speed,
                deltaSpeed=delta_speed,
                currentGap=self.radar_distance,
                maxAcceleration=self.max_acceleration,
                accelerationExponent=self.acceleration_exponent,
                headwayTime=self.headway_time,
                comfortableDeceleration=self.comfortable_deceleration,
                safetyDist=self.safety_distance
            )
            s.data += acc * 0.1  # Update speed using acceleration and timer period (0.1s)
        else:
            # If data is invalid or max_speed is zero, apply gentle deceleration
            if self.gps_speed > 0:
                acc = -self.comfortable_deceleration * 0.5  # Gentle deceleration
            else:
                acc = 0
            s.data += acc * 0.1

        s.data = clamp(s.data, 0.0, self.max_speed)  # Clamp speed to valid range

        s.data = clamp(s.data, 0.0, self.max_speed)
        self.get_logger().info(
            "value %f speed %f dist %f headway time %f idm %f" %
            (s.data, self.gps_speed, self.radar_distance,
             self.radar_distance / self.gps_speed, acc))
        self.current_value = s.data
        self.__speed_publisher.publish(s)


def main(args=None):
    rclpy.init(args=args)
    agent = ACCAgent()
    rclpy.spin(agent)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
