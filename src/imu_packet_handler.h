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
    using HandlerOutput = std::vector<sensor_msgs::Imu>;
    using HandlerType = std::function<HandlerOutput(const ouster::sdk::core::ImuPacket&)>;

   public:
    static HandlerType create(const ouster::sdk::core::SensorInfo& info,
                              const std::string& frame,
                              const std::string& timestamp_mode,
                              int64_t ptp_utc_tai_offset) {
        const auto& pf = ouster::sdk::core::get_format(info);
        using Timestamper = std::function<ros::Time(const ouster::sdk::core::ImuPacket&)>;
        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{
                [](const ouster::sdk::core::ImuPacket& imu_packet) {
                    return impl::ts_to_ros_time(imu_packet.host_timestamp);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [pf, ptp_utc_tai_offset](const ouster::sdk::core::ImuPacket& imu_packet) {
                    if (pf.imu_measurements_per_packet == 0) { // LEGACY
                        auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                        ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                        return impl::ts_to_ros_time(ts);
                    } else {
                        auto ts = imu_packet.timestamp()[0];
                        ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                        return impl::ts_to_ros_time(ts);
                    }
                }};
        } else {
            timestamper = Timestamper{
                [pf](const ouster::sdk::core::ImuPacket& imu_packet) {
                    if (pf.imu_measurements_per_packet == 0) { // LEGACY
                        auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                        return impl::ts_to_ros_time(ts);
                    } else {
                        auto ts = imu_packet.timestamp()[0];
                        return impl::ts_to_ros_time(ts);
                    }
                }};
        }

        return [pf, frame, timestamper, info](const ouster::sdk::core::ImuPacket& imu_packet) {
            return packet_to_imu_msgs(imu_packet, frame, timestamper(imu_packet), info);
        };
    }
};

}  // namespace ouster_ros
