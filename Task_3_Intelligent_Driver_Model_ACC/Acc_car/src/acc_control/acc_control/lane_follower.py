# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""ROS2 Tesla driver."""

import os
import cv2
import numpy as np
from pprint import pprint, pformat
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from rclpy.qos import qos_profile_sensor_data, QoSReliabilityPolicy
from rclpy.node import Node


class LaneFollower(Node):

    def __init__(self):
        print("my Lane Follower")
        super().__init__('lane_follower')

        self.declare_parameter("vehicle", "")
        self.declare_parameter("speed", 20.0)
        self.declare_parameter("gain", 0.0009)
        self.declare_parameter("offset", 0.45)
        self.vehicle_topic = self.get_parameter(
            'vehicle').get_parameter_value().string_value
        self.speed = self.get_parameter(
            'speed').get_parameter_value().double_value
        self.gain = self.get_parameter(
            'gain').get_parameter_value().double_value
        self.offset = self.get_parameter(
            'offset').get_parameter_value().double_value

        # ROS interface
        self.__ackermann_publisher = self.create_publisher(
            AckermannDrive, self.vehicle_topic + '/cmd_ackermann', 1)
        self.__steering_publisher = self.create_publisher(
            Float32, self.vehicle_topic + '/cmd_steering', 1)
        self.__speed_publisher = self.create_publisher(
            Float32, self.vehicle_topic + '/cmd_speed', 1)

        self.command_message = AckermannDrive()
        self.command_message.speed = self.speed
        self.command_message.steering_angle = 0.0

        qos_camera_data = qos_profile_sensor_data
        # In case ROS_DISTRO is Rolling or Galactic the QoSReliabilityPolicy is strict.
        if ('ROS_DISTRO' in os.environ
                and (os.environ['ROS_DISTRO'] == 'rolling'
                     or os.environ['ROS_DISTRO'] == 'galactic')):
            qos_camera_data.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(Image,
                                 self.vehicle_topic + '/camera/image_color',
                                 self.__on_camera_image, qos_camera_data)

    def __on_camera_image(self, message):
        img = message.data
        img = np.frombuffer(img, dtype=np.uint8).reshape(
            (message.height, message.width, 4))

        # Segment the image by color in HSV color space
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        #cv2.imshow("video", img)
        # cv2.waitKey(1)

        mask = cv2.inRange(img, np.array([50, 110, 150]),
                           np.array([120, 255, 255]))

        #cv2.imshow(self.vehicle_topic+" mask", mask)
        # cv2.waitKey(1)

        height, width, channels = img.shape

        border = -1
        for x in range(int(width / 2), 0, -1):
            if img[0, x][0] > 50:
                border = x
                break

        if border > 0:
            # Find error (the lane distance from the target distance)
            error = border - int(width * self.offset)

            self.command_message.steering_angle = error * self.gain
            #self.get_logger().info(
            #    "error %f border %f steering %f" %
            #    (error, border, self.command_message.steering_angle))

        if self.speed > 0:
            self.__ackermann_publisher.publish(self.command_message)
        else:
            s = Float32()
            s.data = self.command_message.steering_angle
            self.__steering_publisher.publish(s)


def main(args=None):
    rclpy.init(args=args)
    follower = LaneFollower()
    rclpy.spin(follower)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
