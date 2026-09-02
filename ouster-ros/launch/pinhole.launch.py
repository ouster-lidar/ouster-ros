# Copyright 2026 John Cameron Furey
# SPDX-License-Identifier: BSD-3-Clause

"""Launch pinhole image panels for an existing Ouster packet stream."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('ouster_ros'))
    default_params = package_share / 'config' / 'os_sensor_cloud_image_params.yaml'

    namespace = LaunchConfiguration('ouster_ns')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('ouster_ns', default_value='ouster'),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params),
            description='parameter file containing the os_pinhole settings',
        ),
        Node(
            package='ouster_ros',
            executable='os_pinhole',
            name='os_pinhole',
            namespace=namespace,
            parameters=[params_file],
            output='screen',
        ),
    ])
