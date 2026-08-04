/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.cpp
 * @brief implementation of OusterProcessingNodeBase interface
 */

#include "ouster_ros/os_processing_node_base.h"

#include <memory>
#include <string>

#include "ouster_ros/impl/file_util.h"

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

bool OusterProcessingNodeBase::load_metadata_from_file(
    const std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>&
        on_sensor_metadata) {
    const std::string meta_file =
        has_parameter("metadata")
            ? get_parameter("metadata").as_string()
            : declare_parameter<std::string>("metadata", "");
    // Treat empty / whitespace-only as "not provided".
    if (meta_file.find_first_not_of(' ') == std::string::npos) return false;

    const std::string contents = impl::read_text_file(meta_file);
    if (contents.empty()) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "metadata file could not be read or is empty: " << meta_file);
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "loaded sensor metadata from file: " << meta_file);
    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = contents;
    on_sensor_metadata(msg);
    return true;
}

}  // namespace ouster_ros
