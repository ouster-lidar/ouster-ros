/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node_base.cpp
 * @brief implementation of OusterSensorNodeBase interface
 */

#include "ouster_ros/os_sensor_node_base.h"

#include <ouster/impl/build.h>

namespace sensor = ouster::sensor;
using ouster_srvs::srv::GetMetadata;
using sensor::UDPProfileLidar;

namespace ouster_ros {

void OusterSensorNodeBase::create_get_metadata_service() {
    get_metadata_srv = create_service<GetMetadata>(
        "get_metadata",
        [this](const std::shared_ptr<GetMetadata::Request>,
               std::shared_ptr<GetMetadata::Response> response) {
            response->metadata = cached_metadata;
            return cached_metadata.size() > 0;
        });

    RCLCPP_INFO(get_logger(), "get_metadata service created");
}

void OusterSensorNodeBase::display_lidar_info(const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    RCLCPP_INFO_STREAM(
        get_logger(),
        "ouster client version: "
            << ouster::SDK_VERSION_FULL << "\n"
            << "product: " << info.prod_line << ", sn: " << info.sn
            << ", firmware rev: " << info.fw_rev << "\n"
            << "lidar mode: " << sensor::to_string(info.mode) << ", "
            << "lidar udp profile: " << sensor::to_string(lidar_profile));
}

}  // namespace ouster_ros
