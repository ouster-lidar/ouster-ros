/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.cpp
 * @brief implementation of OusterProcessingNodeBase interface
 */

#include "ouster_ros/os_processing_node_base.h"

using ouster::sensor::UDPProfileLidar;

namespace ouster_ros {

void OusterProcessingNodeBase::create_metadata_subscriber(
    std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>
        on_sensor_metadata) {
    auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latching_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    latching_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    metadata_sub = create_subscription<std_msgs::msg::String>(
        "metadata", latching_qos, on_sensor_metadata);
}

int OusterProcessingNodeBase::get_n_returns() const {
    return info.format.udp_profile_lidar ==
                   UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
               ? 2
               : 1;
}

}  // namespace ouster_ros
