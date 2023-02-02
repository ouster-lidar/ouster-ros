# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with..."""

from pathlib import Path
import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components in a single
    process/container.
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    params = Path(ouster_ros_pkg_dir) / 'config' / 'parameters.yaml'

    os_container = ComposableNodeContainer(
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

    viz_launch_config = LaunchConfiguration('viz')
    viz_launch_arg = DeclareLaunchArgument('viz', default_value='True')
    viz_launch_file_path = \
        Path(ouster_ros_pkg_dir) / 'launch' / 'sensor.rviz.launch.py'
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(viz_launch_file_path)]),
        condition=IfCondition(viz_launch_config)
    )

    # NOTE: this is rather a workaround to let rviz2 get going and not complain while waiting
    #     for the actual sensor frames to be published that is when when using the same launch file
    #     to run rviz2
    sensor_imu_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       condition=IfCondition(viz_launch_config),
                       arguments = ["0", "0", "0", "0", "0", "0", "os_sensor", "os_imu"])
    sensor_lidar_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       condition=IfCondition(viz_launch_config),
                       arguments = ["0", "0", "0", "0", "0", "0", "os_sensor", "os_lidar"])


    return launch.LaunchDescription([
        os_container,
        viz_launch_arg, sensor_imu_tf, sensor_lidar_tf, rviz_launch])
