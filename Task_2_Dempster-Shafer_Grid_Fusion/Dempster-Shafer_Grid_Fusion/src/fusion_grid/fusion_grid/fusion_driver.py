"""ROS2 Fusion driver."""

from turtle import rt
import rclpy
from pprint import pprint, pformat
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class FusionDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # for the Display in webots
        self.__display = self.__robot.getDevice("grid")

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('fusion_driver')
        self.__node.create_subscription(Image, "myvehicle/grid",
                                        self.__on_grid,
                                        qos_profile_sensor_data)

    def __on_grid(self, msg):
        self.__node.get_logger().info("got grid")
        ir = self.__display.imageNew(msg.data.tobytes(), self.__display.BGRA,
                                     msg.width, msg.height)
        self.__display.imagePaste(ir, 0, 0, False)
        self.__display.imageDelete(ir)

    def step(self):
        for s in range(10):
            rclpy.spin_once(self.__node, timeout_sec=0)
