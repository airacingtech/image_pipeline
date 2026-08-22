#!/usr/bin/env python
from glob import glob
import os

from setuptools import setup, find_packages

PACKAGE_NAME = "camera_calibration"

setup(
    name=PACKAGE_NAME,
    version='5.0.13',
    packages=["camera_calibration", "camera_calibration.nodes"],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        (os.path.join('share', PACKAGE_NAME, 'config'), glob('config/*')),
        (os.path.join('share', PACKAGE_NAME, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME, 'web'), glob('web/*')),
    ],
    py_modules=[],
    package_dir={'': 'src'},
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    author='James Bowman, Patrick Mihelich',
    maintainer='Vincent Rabaud, Steven Macenski',
    maintainer_email='vincent.rabaud@gmail.com, stevenmacenski@gmail.com',
    keywords=['ROS2'],
    description='Camera_calibration allows easy calibration of monocular or stereo cameras using a checkerboard calibration target .',
    license='BSD',
    tests_require=[
        'pytest',
        'requests'
    ],
    entry_points={
        'console_scripts': [
            'cameracalibrator = camera_calibration.nodes.cameracalibrator:main',
            'cameracheck = camera_calibration.nodes.cameracheck:main',
            'tarfile_calibration = camera_calibration.nodes.tarfile_calibration:main',
            'art_camera_calibrator = camera_calibration.nodes.art_camera_calibrator:main',
            'art_calibration_ui = camera_calibration.nodes.art_calibration_ui:main',
            'art_foxglove_relay = camera_calibration.nodes.art_foxglove_relay:main',
            'art_stereo_auto = camera_calibration.nodes.art_stereo_auto:main',
            'art_stereo_capture = camera_calibration.nodes.art_stereo_capture:main',
            'art_stereo_calibrate = camera_calibration.nodes.art_stereo_calibrate:main',
            'art_stereo_status = camera_calibration.nodes.art_stereo_status:main',
        ],
    },
)
