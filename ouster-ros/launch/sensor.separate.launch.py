# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    params = Path(ouster_ros_pkg_dir) / 'config' / 'parameters.yaml'

    os_sensor = Node(
        name='os_sensor',
        namespace='',
        package='ouster_ros',
        executable='os_sensor',
        parameters=[params],
        output='screen',
    )

    os_cloud = Node(
        name='os_cloud',
        namespace='',
        package='ouster_ros',
        executable='os_cloud',
        parameters=[params],
        output='screen',
    )

    os_image = Node(
        name='os_image',
        namespace='',
        package='ouster_ros',
        executable='os_image',
        output='screen',
    )

    return launch.LaunchDescription([
        os_sensor, os_cloud, os_image])
