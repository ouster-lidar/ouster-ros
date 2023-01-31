# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    params = Path(ouster_ros_pkg_dir) / 'config' / 'parameters.yaml'

    container = ComposableNodeContainer(
        name='os_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
                ComposableNode(
                    package='ouster_ros',
                    plugin='ouster_ros::OusterSensor',
                    name='os_sensor',
                    parameters=[params]
                ),
            ComposableNode(
                    package='ouster_ros',
                    plugin='ouster_ros::OusterCloud',
                    name='os_cloud',
                    parameters=[params]
                ),
            ComposableNode(
                    package='ouster_ros',
                    plugin='ouster_ros::OusterImage',
                    name='os_image'
                )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
