/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file telemetry_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class TelemetryHandler {
   public:
    using HandlerOutput = ouster_sensor_msgs::msg::Telemetry;
    using HandlerType = std::function<HandlerOutput(const ouster::sdk::core::LidarPacket&)>;

    static uint64_t first_valid_ts(const ouster::sdk::core::LidarPacket& lidar_packet,
                                   const ouster::sdk::core::PacketFormat& pf) {
        // scan for the first non-zero column ts
        uint64_t ts = 0;
        for (int icol = 0; icol < pf.columns_per_packet; ++icol) {
            ts = pf.col_timestamp(pf.nth_col(icol, lidar_packet.buf.data()));
            if (ts != 0)
                break;
        }
        return ts;
    }

   public:
    static HandlerType create(const ouster::sdk::core::SensorInfo& info,
                              const std::string& timestamp_mode,
                              int64_t ptp_utc_tai_offset) {
        const auto& pf = ouster::sdk::core::get_format(info);
        using Timestamper = std::function<rclcpp::Time(const ouster::sdk::core::LidarPacket&)>;

        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{
                [](const ouster::sdk::core::LidarPacket& lidar_packet) {
                    return rclcpp::Time(lidar_packet.host_timestamp);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [pf, ptp_utc_tai_offset](const ouster::sdk::core::LidarPacket& lidar_packet) {
                    uint64_t ts = TelemetryHandler::first_valid_ts(lidar_packet, pf);
                    ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                    return rclcpp::Time(ts);

                }};
        } else {
            timestamper = Timestamper{
                [pf](const ouster::sdk::core::LidarPacket& lidar_packet) {
                    uint64_t ts = TelemetryHandler::first_valid_ts(lidar_packet, pf);
                    return rclcpp::Time(ts);
                }};
        }

        return [pf, timestamper](const ouster::sdk::core::LidarPacket& lidar_packet) {
            return lidar_packet_to_telemetry_msg(lidar_packet, timestamper(lidar_packet), pf);
        };
    }
};

}  // namespace ouster_ros