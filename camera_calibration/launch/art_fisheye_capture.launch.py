#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


CAMERA_CALIBRATION_SHARE = get_package_share_directory('camera_calibration')
DEFAULT_IAC_CONFIG = os.path.join(
    get_package_share_directory('iac_launch'), 'param', 'cameras_param')
CAMERAS = (
    ('front', 'cam_front_calib.yaml'),
    ('left', 'cam_left_calib.yaml'),
    ('right', 'cam_right_calib.yaml'),
    ('rear', 'cam_rear_calib.yaml'),
)


def generate_launch_description():
    override_config = LaunchConfiguration('override_config')
    common_overrides = ParameterFile(override_config, allow_substs=True)
    nodes = []
    for camera, calibration in CAMERAS:
        nodes.append(Node(
            package='avt_vimba_camera',
            executable='mono_camera_exec',
            name=f'vimba_{camera}',
            output='screen',
            emulate_tty=True,
            parameters=[
                ParameterFile(
                    os.path.join(
                        DEFAULT_IAC_CONFIG, f'vimba_{camera}.param.yaml'),
                    allow_substs=True,
                ),
                common_overrides,
                {
                    'camera_info_url': (
                        f'package://iac_launch/param/cameras_param/{calibration}'),
                },
            ],
        ))

    return LaunchDescription([
        DeclareLaunchArgument(
            'override_config',
            default_value=os.path.join(
                CAMERA_CALIBRATION_SHARE,
                'config',
                'art_fisheye_vimba_override.yaml',
            ),
        ),
        *nodes,
    ])
