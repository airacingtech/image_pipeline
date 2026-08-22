#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8766'),
        DeclareLaunchArgument('send_buffer_limit', default_value='20000000'),
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_calibration_bridge',
            output='screen',
            parameters=[{
                'address': '0.0.0.0',
                'port': ParameterValue(
                    LaunchConfiguration('port'), value_type=int),
                # Keep only a few 2K frames queued. A large queue increases
                # latency and makes a live preview look disconnected.
                'send_buffer_limit': ParameterValue(
                    LaunchConfiguration('send_buffer_limit'), value_type=int),
            }],
        ),
    ])
