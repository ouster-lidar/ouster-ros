/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file zone_packet_handler.h
 * @brief Handles zone monitoring packets from the Ouster sensor and converts
 * them to ROS messages.
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class ZonePacketHandler {
   public:
    using HandlerOutput = ouster_sensor_msgs::msg::ZoneStatus;
    using HandlerType = std::function<HandlerOutput(const ouster::sdk::core::ZonePacket&)>;

   public:
    static HandlerType create(const ouster::sdk::core::SensorInfo& info,
                              const std::string& frame,
                              const std::string& timestamp_mode,
                              int64_t ptp_utc_tai_offset) {
        using Timestamper = std::function<rclcpp::Time(const ouster::sdk::core::ZonePacket&)>;
        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{
                [](const ouster::sdk::core::ZonePacket& zone_packet) {
                    return rclcpp::Time(zone_packet.host_timestamp);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [ptp_utc_tai_offset](const ouster::sdk::core::ZonePacket& zone_packet) {
                    auto ts = zone_packet.timestamp();
                    ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                    return rclcpp::Time(ts);
                }};
        } else {
            timestamper = Timestamper{
                [](const ouster::sdk::core::ZonePacket& zone_packet) {
                    return rclcpp::Time(zone_packet.timestamp());
                }};
        }

        auto zone_labels = get_zone_labels(info);

        return [frame, timestamper, zone_labels](
                   const ouster::sdk::core::ZonePacket& zone_packet) {
            return zone_packet_to_zone_status_msg(
                zone_packet, frame, timestamper(zone_packet), zone_labels);
        };
    }
};

}  // namespace ouster_ros
