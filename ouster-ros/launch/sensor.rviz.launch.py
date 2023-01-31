# Copyright 2023 Ouster, Inc.
#

from pathlib import Path
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    config_file_path = Path(ouster_ros_pkg_dir) / 'config' / 'viz.rviz'
    return launch.LaunchDescription([
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(config_file_path)]
        )
    ])
