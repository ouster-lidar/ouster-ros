# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import TimerAction

def generate_launch_description():
    """Generate launch description with multiple components."""
    os_sensor_container = ComposableNodeContainer(
            name='os_sensor_container',
            namespace='ouster_ns',
            package='rclcpp_components',
            executable='component_container_mt',
            # prefix=['xterm -fa "Monospace" -fs 14 -e gdb -ex run --args'],    # enable -g in CMakeLists.txt
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
                )
            ],
            output='screen',
    )

    os_cloud_container = ComposableNodeContainer(
            name='os_cloud_container',
            namespace='ouster_ns',
            package='rclcpp_components',
            executable='component_container_mt',
            # prefix=['xterm -fa "Monospace" -fs 14 -e gdb -ex run --args'],    # enable -g in CMakeLists.txt
            composable_node_descriptions=[
                ComposableNode(
                    package='ouster_ros',
                    plugin='nodelets_os::OusterCloud',
                    name='os_cloud',
                    parameters=[
                        {'tf_prefix' : ' '},
                        {'timestamp_mode' : ' '}]
                )
            ],
            output='screen',
    )

    os_cloud_container_delayed = TimerAction(period=10.0, actions=[os_cloud_container])

    os_image_container = ComposableNodeContainer(
            name='os_image_container',
            namespace='ouster_ns',
            package='rclcpp_components',
            executable='component_container_mt',
            # prefix=['xterm -fa "Monospace" -fs 14 -e gdb -ex run --args'],    # enable -g in CMakeLists.txt
            composable_node_descriptions=[
                ComposableNode(
                    package='ouster_ros',
                    plugin='nodelets_os::OusterImage',
                    name='os_image'
                )
            ],
            output='screen',
    )

    os_image_container_delayed = TimerAction(period=10.0, actions=[os_image_container])

    return launch.LaunchDescription([
        os_sensor_container,
        os_cloud_container_delayed,
        os_image_container_delayed])
