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
    using HandlerType = std::function<HandlerOutput(const uint8_t*)>;

   public:
    static HandlerType create_handler(const ouster::sensor::sensor_info& info,
                                      const std::string& frame,
                                      const std::string& timestamp_mode,
                                      int64_t ptp_utc_tai_offset) {
        const auto& pf = ouster::sensor::get_format(info);
        using Timestamper = std::function<rclcpp::Time(const uint8_t*)>;
        Timestamper timestamper;

        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{[](const uint8_t* /*imu_buf*/) {
                return rclcpp::Clock(RCL_ROS_TIME).now();
            }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper =
                Timestamper{[pf, ptp_utc_tai_offset](const uint8_t* imu_buf) {
                    uint64_t ts = pf.imu_gyro_ts(imu_buf);
                    ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                    return rclcpp::Time(ts);
                }};
        } else {
            timestamper = Timestamper{[pf](const uint8_t* imu_buf) {
                return rclcpp::Time(pf.imu_gyro_ts(imu_buf));
            }};
        }

        return [&pf, &frame, timestamper](const uint8_t* imu_buf) {
            return packet_to_imu_msg(pf, timestamper(imu_buf), frame, imu_buf);
        };
    }
};

}  // namespace ouster_ros