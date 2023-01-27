# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='os_container',
            namespace='ouster_ns',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='ouster_ros',
                    plugin='nodelets_os::OusterSensor',
                    name='os_sensor',
                    parameters=[
                        {'sensor_hostname' : 'os-122151001683.local'},
                        {'udp_dest' : ' '},
                        {'lidar_mode' : ' '},
                        {'timestamp_mode' : ' '},
                        {'udp_profile_lidar' : ' '},
                        {'metadata' : ' '},
                        {'lidar_port' : 48853},
                        {'imu_port' : 37028}]
                ),
                ComposableNode(
                    package='ouster_ros',
                    plugin='nodelets_os::OusterCloud',
                    name='os_cloud',
                    parameters=[
                        {'tf_prefix' : ' '},
                        {'timestamp_mode' : ' '}]
                ),
                ComposableNode(
                    package='ouster_ros',
                    plugin='nodelets_os::OusterImage',
                    name='os_image'
                )
            ],
            output='screen',
    )

    return launch.LaunchDescription([container])
