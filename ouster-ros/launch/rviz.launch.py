# Copyright 2023 Ouster, Inc.
#

from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Generate launch description for rviz"""
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    print(ouster_ros_pkg_dir)
    default_rviz_config = Path(ouster_ros_pkg_dir) / 'config' / 'viz.rviz'
    print(default_rviz_config)
    rviz_config = LaunchConfiguration('rviz_config')
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value=str(default_rviz_config))

    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster')

    rviz_node = Node(
        package='rviz2',
        namespace=ouster_ns,
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        ouster_ns_arg,
        rviz_config_arg,
        rviz_node
    ])
