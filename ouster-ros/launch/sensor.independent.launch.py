# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    params = Path(ouster_ros_pkg_dir) / 'config' / 'parameters.yaml'

    # TODO: changing the default namespace is currently not supported when using the launch.py format'
    ouster_ns = LaunchConfiguration('ouster_ns')
    ouster_ns_launch_arg = DeclareLaunchArgument(
        'ouster_ns', default_value='ouster')

    viz_launch_config = LaunchConfiguration('viz')
    viz_launch_arg = DeclareLaunchArgument('viz', default_value='True')

    os_sensor = Node(
        package='ouster_ros',
        executable='os_sensor',
        name='os_sensor',
        namespace=ouster_ns,
        parameters=[params],
        output='screen',
    )

    os_cloud = Node(
        package='ouster_ros',
        executable='os_cloud',
        name='os_cloud',
        namespace=ouster_ns,
        parameters=[params],
        output='screen',
    )

    os_image = Node(
        package='ouster_ros',
        executable='os_image',
        name='os_image',
        namespace=ouster_ns,
        parameters=[params],
        output='screen',
    )

    viz_launch_file_path = \
        Path(ouster_ros_pkg_dir) / 'launch' / 'sensor.rviz.launch.py'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(viz_launch_file_path)]),
        condition=IfCondition(viz_launch_config)
    )

    # NOTE: this is rather a workaround to let rviz2 get going and not complain while waiting
    #     for the actual sensor frames to be published that is when when using the same launch file
    #     to run rviz2
    sensor_imu_tf = Node(package="tf2_ros",
                         executable="static_transform_publisher",
                         name="stp_sensor_imu",
                         namespace=ouster_ns,
                         condition=IfCondition(viz_launch_config),
                         arguments=["0", "0", "0", "0", "0", "0", "os_sensor", "os_imu"])
    sensor_ldr_tf = Node(package="tf2_ros",
                         executable="static_transform_publisher",
                         name="stp_sensor_lidar",
                         namespace=ouster_ns,
                         condition=IfCondition(viz_launch_config),
                         arguments=["0", "0", "0", "0", "0", "0", "os_sensor", "os_lidar"])

    return launch.LaunchDescription([
        ouster_ns_launch_arg,
        os_sensor, os_cloud, os_image,
        viz_launch_arg, sensor_imu_tf, sensor_ldr_tf, rviz_launch])
