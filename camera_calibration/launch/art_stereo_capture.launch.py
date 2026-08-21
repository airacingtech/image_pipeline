#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


CAMERA_CALIBRATION_SHARE = get_package_share_directory('camera_calibration')
DEFAULT_IAC_CONFIG = os.path.join(
    get_package_share_directory('iac_launch'), 'param', 'cameras_param')


def generate_launch_description():
    left_config = LaunchConfiguration('left_config')
    right_config = LaunchConfiguration('right_config')
    override_config = LaunchConfiguration('override_config')
    trigger_config = LaunchConfiguration('trigger_config')

    common_overrides = ParameterFile(override_config, allow_substs=True)

    left = Node(
        package='avt_vimba_camera',
        executable='mono_camera_exec',
        name='vimba_calib_left',
        output='screen',
        emulate_tty=True,
        parameters=[
            ParameterFile(left_config, allow_substs=True),
            common_overrides,
            {
                'name': 'camera',
                'frame_id': 'vimba_front_left_center',
                'ip': '10.42.17.53',
                'guid': '',
                # Never load the previous monocular or wrong-size CameraInfo
                # while collecting a new stereo dataset.
                'camera_info_url': '',
            },
        ],
    )

    right = Node(
        package='avt_vimba_camera',
        executable='mono_camera_exec',
        name='vimba_calib_right',
        output='screen',
        emulate_tty=True,
        parameters=[
            ParameterFile(right_config, allow_substs=True),
            common_overrides,
            {
                'name': 'camera',
                'frame_id': 'vimba_front_right_center',
                'ip': '10.42.17.51',
                'guid': '',
                'camera_info_url': '',
            },
        ],
    )

    trigger = Node(
        package='avt_vimba_camera',
        executable='trigger_exec',
        name='vimba_calibration_trigger',
        output='screen',
        emulate_tty=True,
        parameters=[ParameterFile(trigger_config, allow_substs=True)],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'left_config',
            default_value=os.path.join(
                DEFAULT_IAC_CONFIG, 'vimba_front_left_center.param.yaml'),
        ),
        DeclareLaunchArgument(
            'right_config',
            default_value=os.path.join(
                DEFAULT_IAC_CONFIG, 'vimba_front_right_center.param.yaml'),
        ),
        DeclareLaunchArgument(
            'override_config',
            default_value=os.path.join(
                CAMERA_CALIBRATION_SHARE, 'config', 'art_stereo_vimba_override.yaml'),
        ),
        DeclareLaunchArgument(
            'trigger_config',
            default_value=os.path.join(
                CAMERA_CALIBRATION_SHARE, 'config', 'art_stereo_trigger.yaml'),
        ),
        left,
        right,
        # Allow both GigE cameras to enter AcquisitionMode before triggering.
        TimerAction(period=5.0, actions=[trigger]),
    ])
