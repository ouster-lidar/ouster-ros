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

    enable_static_tf = LaunchConfiguration('_enable_static_tf_publishers')
    enable_static_tf_arg = DeclareLaunchArgument(
        '_enable_static_tf_publishers', default_value='false')

    # NOTE: the two static tf publishers are rather a workaround to let rviz2
    #     get going and not complain while waiting for the actual sensor frames
    #     to be published that is when running rviz2 using a parent launch file
    # TODO: need to be able to propagate the modified frame names from the
    #       parameters file to RVIZ launch py.
    sensor_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="stp_sensor_imu",
        namespace=ouster_ns,
        condition=IfCondition(enable_static_tf),
        arguments=["--frame-id", "os_sensor", "--child-frame-id", "os_imu"])
    sensor_ldr_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="stp_sensor_lidar",
        namespace=ouster_ns,
        condition=IfCondition(enable_static_tf),
        arguments=["--frame-id", "os_sensor", "--child-frame-id", "os_lidar"])

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
        enable_static_tf_arg,
        sensor_imu_tf,
        sensor_ldr_tf,
        rviz_node
    ])
