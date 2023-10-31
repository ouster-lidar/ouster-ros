/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node_base.cpp
 * @brief implementation of OusterSensorNodeBase interface
 */

#include "ouster_ros/os_sensor_node_base.h"

#include <ouster/impl/build.h>

#include <fstream>

namespace sensor = ouster::sensor;
using ouster_sensor_msgs::srv::GetMetadata;
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

void OusterSensorNodeBase::create_metadata_publisher() {
    auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latching_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    latching_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    metadata_pub =
        create_publisher<std_msgs::msg::String>("metadata", latching_qos);
}

void OusterSensorNodeBase::publish_metadata() {
    std_msgs::msg::String metadata_msg;
    metadata_msg.data = cached_metadata;
    metadata_pub->publish(metadata_msg);
}

void OusterSensorNodeBase::display_lidar_info(const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    RCLCPP_INFO_STREAM(
        get_logger(),
        "ouster client version: "
            << ouster::SDK_VERSION_FULL << "\n"
            << "product: " << info.prod_line << ", sn: " << info.sn << ", "
            << "firmware rev: " << info.fw_rev << "\n"
            << "lidar mode: " << sensor::to_string(info.mode) << ", "
            << "lidar udp profile: " << sensor::to_string(lidar_profile));
}

std::string OusterSensorNodeBase::read_text_file(const std::string& text_file) {
    std::ifstream ifs{};
    ifs.open(text_file);
    if (ifs.fail()) return {};
    std::stringstream buf;
    buf << ifs.rdbuf();
    return buf.str();
}

bool OusterSensorNodeBase::write_text_to_file(const std::string& file_path,
                                              const std::string& text) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) return false;
    ofs << text << std::endl;
    ofs.close();
    return true;
}

}  // namespace ouster_ros
