#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController

from robot_launch.ros2_nodes import get_ros2_nodes

def generate_launch_description():
    package_dir = get_package_share_directory('robot_launch')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(world=PathJoinSubstitution(
        [package_dir, 'worlds', world]),
                            ros2_supervisor=False,
                            gui=True,
                            mode="realtime",
                            stream=False)

    supervisor = Ros2SupervisorLauncher()

    robot_description_path = os.path.join(package_dir, 'resource',
                                          'robot_webots.urdf')
    
    vehicle_driver = WebotsController(robot_name='myrobot',
                                    parameters=[{
                                        'robot_description':
                                        robot_description_path
                                    }],
                                    respawn=True)



    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=supervisor,
            on_exit=get_ros2_nodes,
        ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value='tesla_world.wbt', description=''),
        DeclareLaunchArgument('testdefinition', default_value=''),
        DeclareLaunchArgument('testsuite', default_value=''),
        DeclareLaunchArgument('testcase', default_value=''),
        webots,
        supervisor,

        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.
        RegisterEventHandler(event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )),
        # Add the reset event handler
        reset_handler,
        vehicle_driver,
    ] + get_ros2_nodes())
