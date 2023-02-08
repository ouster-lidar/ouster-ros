/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.cpp
 * @brief implementation of OusterClientBase interface
 */

#include "ouster_ros/os_client_base_nodelet.h"

#include <ouster/impl/build.h>
#include "ouster_ros/GetMetadata.h"

namespace sensor = ouster::sensor;
using sensor::UDPProfileLidar;
using ouster_ros::GetMetadata;

namespace nodelets_os {

void OusterClientBase::onInit() {
    auto& nh = getNodeHandle();
    get_metadata_srv =
        nh.advertiseService<GetMetadata::Request, GetMetadata::Response>(
            "get_metadata",
            [this](GetMetadata::Request&, GetMetadata::Response& res) {
                res.metadata = cached_metadata;
                return cached_metadata.size() > 0;
            });

    NODELET_INFO("get_metadata service created");
}

void OusterClientBase::display_lidar_info(const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    auto n_returns =
        lidar_profile == UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
            ? 2
            : 1;
    NODELET_INFO_STREAM(
        "ouster client version: "
            << ouster::SDK_VERSION_FULL << "\n"
            << "using lidar_mode: " << sensor::to_string(info.mode) << "\n"
            << "product: " << info.prod_line << ", sn: " << info.sn
            << ", firmware rev: " << info.fw_rev << "\n"
            << "active profile: " << sensor::to_string(lidar_profile) << "\n"
            << "profile has " << n_returns << " return(s)");

}

}  // namespace nodelets_os
