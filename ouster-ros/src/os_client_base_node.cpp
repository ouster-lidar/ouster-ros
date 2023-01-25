/**
 * Copyright (c) 2018-2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_node.cpp
 * @brief implementation of OusterClientBase interface
 */

#include "ouster_ros/os_client_base_node.h"

#include <ouster/impl/build.h>


namespace sensor = ouster::sensor;
using ouster_ros::srv::GetMetadata;

namespace nodelets_os {

void OusterClientBase::onInit() {
    get_metadata_srv = create_service<GetMetadata>(
        "get_metadata",
        [this](
            const std::shared_ptr<GetMetadata::Request>,
            std::shared_ptr<GetMetadata::Response> response) {
            response->metadata = cached_metadata;
            return cached_metadata.size() > 0;
        });

    RCLCPP_INFO(get_logger(), "get_metadata service created");
}

void OusterClientBase::display_lidar_info(const sensor::sensor_info& info) {
    RCLCPP_INFO(get_logger(), "Client version: %s", ouster::SDK_VERSION_FULL);
    RCLCPP_INFO(get_logger(), "Using lidar_mode: %s", sensor::to_string(info.mode).c_str());
    RCLCPP_INFO(get_logger(), "%s sn: %s firmware rev: %s", info.prod_line.c_str(),
                 info.sn.c_str(), info.fw_rev.c_str());
}

}  // namespace nodelets_os
