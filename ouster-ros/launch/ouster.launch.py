#!/usr/bin/python3
# Copyright 2023 Ouster, Inc.

import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    NAMESPACE_ARG = DeclareLaunchArgument(
    "namespace",
    description="(str) namespace that will contain the pipeline nodes and topics",
    default_value="none",
    )

    NAMESPACE = LaunchConfiguration("namespace", default="none")

    PARAMS_ARG = DeclareLaunchArgument(
    "params",
    description="(str) Path to the yaml file containing the parameters for the nodes in "
    "the launch file.",
    default_value="",
    )

    PARAMS = LaunchConfiguration("params", default="")

    os_sensor = LifecycleNode(
        package="ouster_ros",
        executable="os_sensor",
        name="os_sensor",
        namespace=NAMESPACE,
        parameters=[PARAMS],
        respawn=True,
        respawn_delay=4,
    )

    sensor_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(os_sensor),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    sensor_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=os_sensor,
            goal_state="inactive",
            entities=[
                LogInfo(msg="os_sensor activating..."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(os_sensor),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
            handle_once=True,
        )
    )

    os_cloud = Node(
        package="ouster_ros",
        executable="os_cloud",
        name="os_cloud",
        namespace=NAMESPACE,
        parameters=[PARAMS],
        respawn=True,
        respawn_delay=4,
    )

    os_image = Node(
        package="ouster_ros",
        executable="os_image",
        name="os_image",
        namespace=NAMESPACE,
        parameters=[PARAMS],
        remappings=[
            ([NAMESPACE, "/signal_image"], [NAMESPACE, "/intensity_image"]),
            ([NAMESPACE, "/signal_image2"], [NAMESPACE, "/intensity_image2"]),
            ([NAMESPACE, "/nearir_image"], [NAMESPACE, "/ambient_image"]),
            ([NAMESPACE, "/nearir_image2"], [NAMESPACE, "/ambient_image2"]),
        ],
        respawn=True,
        respawn_delay=4,
    )

    return LaunchDescription(
        [
            NAMESPACE_ARG,
            PARAMS_ARG,
            os_sensor,
            os_cloud,
            os_image,
            sensor_configure_event,
            sensor_activate_event,
        ]
    )
