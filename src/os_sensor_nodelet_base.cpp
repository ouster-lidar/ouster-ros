/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_nodelet_base.cpp
 * @brief implementation of OusterSensorNodeletBase interface
 */

#include "ouster_ros/os_sensor_nodelet_base.h"

#include <ouster/impl/build.h>
#include <std_msgs/String.h>

#include <fstream>

#include "ouster_ros/GetMetadata.h"

namespace sensor = ouster::sensor;

namespace ouster_ros {

void OusterSensorNodeletBase::create_get_metadata_service() {
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

void OusterSensorNodeletBase::create_metadata_pub() {
    auto& nh = getNodeHandle();
    metadata_pub = nh.advertise<std_msgs::String>("metadata", 1, true);
}

void OusterSensorNodeletBase::publish_metadata() {
    std_msgs::String metadata_msg;
    metadata_msg.data = cached_metadata;
    metadata_pub.publish(metadata_msg);
}

void OusterSensorNodeletBase::display_lidar_info(
    const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    NODELET_INFO_STREAM(
        "ouster client version: "
        << ouster::SDK_VERSION_FULL << "\n"
        << "product: " << info.prod_line << ", sn: " << info.sn
        << ", firmware rev: " << info.fw_rev << "\n"
        << "lidar mode: " << sensor::to_string(info.mode) << ", "
        << "lidar udp profile: " << sensor::to_string(lidar_profile));
}

std::string OusterSensorNodeletBase::read_text_file(
    const std::string& text_file) {
    std::ifstream ifs{};
    ifs.open(text_file);
    if (ifs.fail()) return {};
    std::stringstream buf;
    buf << ifs.rdbuf();
    return buf.str();
}

bool OusterSensorNodeletBase::write_text_to_file(const std::string& file_path,
                                                 const std::string& text) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) return false;
    ofs << text << std::endl;
    ofs.close();
    return true;
}

}  // namespace ouster_ros
