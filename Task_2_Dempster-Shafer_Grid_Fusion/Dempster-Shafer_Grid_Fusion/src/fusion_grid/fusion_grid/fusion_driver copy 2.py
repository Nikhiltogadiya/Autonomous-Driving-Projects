# fusion_driver.py
"""ROS2 Fusion driver."""

import rclpy
# from pprint import pprint, pformat # Not used
from rclpy.qos import qos_profile_sensor_data # Can be used for subscriber if needed
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class FusionDriver:

    def __init__(self, webots_node, properties): # CORRECTED: __init__
        """
        Constructor for the FusionDriver.
        `webots_node` is the ROS2 node instance provided by webots_ros2_driver.
        """
        self.__node = webots_node  # USE the webots_node provided by the framework
        self.__robot = self.__node.robot
        self.bridge = CvBridge()   # Initialize CvBridge here

        self.__node.get_logger().info("FusionDriver controller initializing...")

        # 1. Display device setup
        self.display_device_name = 'navidisplay' # Verify this matches your Webots robot's Display DEF name
        try:
            self.__display = self.__robot.getDevice(self.display_device_name)
            self.__node.get_logger().info(f"Successfully accessed Display device: '{self.display_device_name}'")
        except Exception as e:
            self.__node.get_logger().error(f"Failed to get Display device '{self.display_device_name}'. Error: {e}")
            self.__display = None
            return # Stop initialization if display is not found

        # 2. Grid map subscription setup
        self.grid_topic_name = '/myrobot/grid_ds' # Topic for the Dempster-Shafer grid
        self.img_subscriber = self.__node.create_subscription( # Use self.__node.create_subscription
            Image,
            self.grid_topic_name,
            self.navidisplay_callback, # Callback to process and display the image
            qos_profile_sensor_data # Using sensor data QoS profile
        )
        self.__node.get_logger().info(f"Fusion driver subscribed to Display '{self.display_device_name}' and topic '{self.grid_topic_name}'")
        
        # 3. Temporary directory for image saving
        self.temp_image_dir = "/tmp"
        if not os.path.exists(self.temp_image_dir):
            try:
                os.makedirs(self.temp_image_dir)
            except OSError as e:
                self.__node.get_logger().error(f"Failed to create temp directory {self.temp_image_dir}: {e}")
                self.temp_image_dir = "." # Fallback

        # REMOVED the following problematic lines:
        # rclpy.init(args=None)  # DO NOT call rclpy.init() here
        # self.__node = rclpy.create_node('fusion_driver') # DO NOT create a new node here
        # REMOVED the conflicting subscription to "myrobot/grid" and __on_grid method
        # self.__node.create_subscription(Image, "myrobot/grid", self.__on_grid, qos_profile_sensor_data)


    def navidisplay_callback(self, ros_image_msg):
        if self.__display is None:
            # self.__node.get_logger().warn("Display device not available, skipping image update.") # Already logged if init failed
            return

        self.__node.get_logger().debug(f"Received image on topic {self.grid_topic_name}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(ros_image_msg, desired_encoding='bgra8')
        except CvBridgeError as e:
            self.__node.get_logger().error(f'CvBridge Error: {e}')
            return
        except Exception as e:
            self.__node.get_logger().error(f'Error converting image: {e}')
            return

        if cv_image is None:
            self.__node.get_logger().error("cv_image is None after conversion.")
            return

        temp_image_path = os.path.join(self.temp_image_dir, "webots_display_grid.png")

        try:
            success = cv2.imwrite(temp_image_path, cv_image)
            if not success:
                self.__node.get_logger().error(f"Failed to save temporary image to {temp_image_path}")
                return
        except Exception as e:
            self.__node.get_logger().error(f"Error saving temporary image to {temp_image_path}: {e}")
            return
        
        try:
            self.__display.imageLoad(temp_image_path)
            # self.__node.get_logger().debug(f"Loaded image {temp_image_path} into display.") # Can be noisy
        except Exception as e:
            self.__node.get_logger().error(f"Error loading image into Webots display from {temp_image_path}: {e}")

    # REMOVED __on_grid method as it's not used with the corrected subscription logic.
    # def __on_grid(self, msg):
    #     ...

    def step(self):
        """
        This method is called by the Webots ROS2 driver at each simulation step.
        It's responsible for processing ROS2 events for this controller's node.
        """
        if not hasattr(self, '_FusionDriver__node') or self.__node is None:
             # This check is a safeguard, but __init__ should always set self.__node if called.
             # If __init__ failed catastrophically or wasn't called, this prevents further errors in step.
             # print("Error: self.__node not initialized in FusionDriver. Step method cannot proceed.", flush=True)
             return

        try:
            # This is the line from your traceback (line 119 or similar)
            rclpy.spin_once(self.__node, timeout_sec=0)
        except Exception as e:
            if self.__node: # Check if node exists before trying to use its logger
                self.__node.get_logger().error(f"Error in rclpy.spin_once during step: {e}")
            # else:
                # print(f"Critical error in step: self.__node is None. Exception: {e}", flush=True)