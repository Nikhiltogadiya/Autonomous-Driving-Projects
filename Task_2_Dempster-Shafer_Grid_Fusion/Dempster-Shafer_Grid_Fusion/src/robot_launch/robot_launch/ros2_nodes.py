#!/usr/bin/env python

import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController


def get_ros2_nodes(*args):
    testconfig = Node(
        package='robot_test',
        executable='testconfiguration',
        output='screen',
        parameters=[
            {
                'servername': "webots_vehicle_node",
                'testdefinition': launch.substitutions.LaunchConfiguration('testdefinition'),
                'testsuite': launch.substitutions.LaunchConfiguration('testsuite'),
                'testcase': launch.substitutions.LaunchConfiguration('testcase')
            },
        ],
    )

    fusion_grid = Node(
        package="fusion_grid",
        #package="fusion_grid_cpp",
        executable='grid',
        output='screen',
        parameters=[
            {
                'vehicle': 'myrobot',
            },
        ],
    )


    return [
        testconfig,
        fusion_grid
    ]
