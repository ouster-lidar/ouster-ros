/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file imu_packet_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class ImuPacketHandler {
   public:
    using HandlerOutput = sensor_msgs::msg::Imu;
    using HandlerType = std::function<HandlerOutput(const sensor::ImuPacket&)>;

   public:
    static HandlerType create(const sensor::sensor_info& info,
                              const std::string& frame,
                              const std::string& timestamp_mode,
                              int64_t ptp_utc_tai_offset) {
        const auto& pf = sensor::get_format(info);
        using Timestamper = std::function<rclcpp::Time(const sensor::ImuPacket&)>;
        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{
                [](const sensor::ImuPacket& imu_packet) {
                    return rclcpp::Time(imu_packet.host_timestamp);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [pf, ptp_utc_tai_offset](const sensor::ImuPacket& imu_packet) {
                    auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                    ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                    return rclcpp::Time(ts);
                }};
        } else {
            timestamper = Timestamper{
                [pf](const sensor::ImuPacket& imu_packet) {
                    auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                    return rclcpp::Time(ts);
                }};
        }

        return [&pf, &frame, timestamper](const sensor::ImuPacket& imu_packet) {
            return packet_to_imu_msg(pf, timestamper(imu_packet), frame,
                                     imu_packet.buf.data());
        };
    }
};

}  // namespace ouster_ros