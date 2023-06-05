# Copyright 2023 Ouster, Inc.
#

"""Launch a sensor node along with os_cloud and os_"""

from pathlib import Path
import launch
import lifecycle_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, LifecycleNode
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, EmitEvent, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():
    """
    Generate launch description for running ouster_ros components separately each
    component will run in a separate process).
    """
    ouster_ros_pkg_dir = get_package_share_directory('ouster_ros')
    # use the community_driver_config.yaml by default
    default_params_file = \
        Path(ouster_ros_pkg_dir) / 'config' / 'community_driver_config.yaml'
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument('params_file',
                                            default_value=str(
                                                default_params_file),
                                            description='name or path to the parameters file to use.')

    driver_launch_file_path = \
        Path(ouster_ros_pkg_dir) / 'launch' / 'driver.launch.py'
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([str(driver_launch_file_path)]),
        launch_arguments={
            'params_file': params_file,
            'ouster_ns': '',
            'os_driver_name': 'ouster_driver',
            'viz': 'True',
            'rviz_config': './install/ouster_ros/share/ouster_ros/config/community_driver.rviz'
        }.items()
    )

    return launch.LaunchDescription([
        params_file_arg,
        driver_launch
    ])
