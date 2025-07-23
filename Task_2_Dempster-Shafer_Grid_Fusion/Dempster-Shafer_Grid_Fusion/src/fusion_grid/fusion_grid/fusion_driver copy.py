"""ROS2 Fusion driver."""

from turtle import rt
import rclpy
from pprint import pprint, pformat
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError # <<< ADD CvBridgeError
import cv2                                    # <<< ADD cv2
import os                                     # <<< ADD os for temporary file path


class FusionDriver:

    def init(self, webots_node, properties):
        self.__robot = webots_node.robot

        # for the Display in webots
        # self.__display = self.__robot.getDevice("grid") # old default
        # 1. VERIFY THIS DEVICE NAME: Check your robot's Display device name in Webots
        self.display_device_name = 'navidisplay' # Or 'display', 'mapdisplay', etc.
        try:
            self.__display = self.__robot.getDevice(self.display_device_name)
            self.get_logger().info(f"Successfully accessed Display device: '{self.display_device_name}'")
            # You might want to set the display color and alpha here if not done elsewhere
            # self.__display.setColor(0xFFFFFF) # White background for clarity before image load
            # self.__display.setAlpha(1.0)
        except Exception as e:
            self.get_logger().error(f"Failed to get Display device '{self.display_device_name}'. Error: {e}")
            self.__display = None
            return
        
        
        
        # self.img_subscriber = self.create_subscription(Image, '/myrobot/grid', self.navidisplay_callback, 1) # default old
        # 2. CORRECTED TOPIC NAME: Must match the publisher in lidar_grid.py
        self.grid_topic_name = '/myrobot/grid_ds' # Corrected from '/myrobot/grid'
        self.img_subscriber = self.create_subscription(
            Image,
            self.grid_topic_name,
            self.navidisplay_callback,
            1  # QoS profile, consider qos_profile_sensor_data if issues
        )
        self.get_logger().info(f"Fusion driver subscribed to Display '{self.display_device_name}' and topic '{self.grid_topic_name}'")
        
        # Directory for temporary image file
        self.temp_image_dir = "/tmp"
        if not os.path.exists(self.temp_image_dir):
            try:
                os.makedirs(self.temp_image_dir)
            except OSError as e:
                self.get_logger().error(f"Failed to create temp directory {self.temp_image_dir}: {e}")
                self.temp_image_dir = "." # Fallback to current directory

        # ROS interface
        rclpy.init(args=None)
        self.__node = rclpy.create_node('fusion_driver')
        self.__node.create_subscription(Image, "myrobot/grid",
                                        self.__on_grid,
                                        qos_profile_sensor_data)

    # def navidisplay_callback(self, img):
    #     self.__display.imageSave(None, "/tmp/display_image.png")
    def navidisplay_callback(self, ros_image_msg):
        if self.__display is None:
            self.get_logger().warn("Display device not available, skipping image update.")
            return

        self.get_logger().debug(f"Received image on topic {self.grid_topic_name}")
        try:
            # Convert ROS Image message to OpenCV image (BGRA8 format is expected from lidar_grid.py)
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgra8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        if cv_image is None:
            self.get_logger().error("cv_image is None after conversion.")
            return

        # To use self.__display.imageLoad(), we need to save the cv_image to a temporary file.
        # Webots typically supports PNG for imageLoad.
        temp_image_path = os.path.join(self.temp_image_dir, "webots_display_grid.png")

        try:
            # Save the OpenCV image (which is BGRA) to a PNG file.
            # cv2.imwrite handles conversion from BGRA to a format suitable for PNG.
            success = cv2.imwrite(temp_image_path, cv_image)
            if not success:
                self.get_logger().error(f"Failed to save temporary image to {temp_image_path}")
                return
        except Exception as e:
            self.get_logger().error(f"Error saving temporary image to {temp_image_path}: {e}")
            return

        # Load the saved image file into the Webots display device
        try:
            # imageLoad might return an error code or raise an exception on failure
            # For older Webots versions, it might not return anything.
            self.__display.imageLoad(temp_image_path)
            self.get_logger().debug(f"Loaded image {temp_image_path} into display.")
        except Exception as e:
            self.get_logger().error(f"Error loading image into Webots display from {temp_image_path}: {e}")



    def __on_grid(self, msg):
        self.__node.get_logger().info("got grid")
        ir = self.__display.imageNew(msg.data.tobytes(), self.__display.BGRA,
                                     msg.width, msg.height)
        self.__display.imagePaste(ir, 0, 0, False)
        self.__display.imageDelete(ir)

    def step(self):
        for s in range(10):
            rclpy.spin_once(self.__node, timeout_sec=0)
