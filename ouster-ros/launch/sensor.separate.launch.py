# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

import launch
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """Generate launch description for ouster_ros."""
    os_sensor = Node(
        name='os_sensor',
        namespace='',
        package='ouster_ros',
        executable='os_sensor',
        parameters=[
                {'sensor_hostname': 'os-122151001683.local'},
                {'udp_dest': ' '},
                {'lidar_mode': ' '},
                {'timestamp_mode': ' '},
                {'udp_profile_lidar': ' '},
                {'metadata': ' '},
                {'lidar_port': 48853},
                {'imu_port': 37028}],
        output='screen',
    )

    os_cloud = Node(
        name='os_cloud',
        namespace='',
        package='ouster_ros',
        executable='os_cloud',
        parameters=[
            {'tf_prefix': ' '},
            {'timestamp_mode': ' '}],
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
        os_sensor,
        TimerAction(period=3.0, actions=[os_cloud]),
        TimerAction(period=4.0, actions=[os_image])])
