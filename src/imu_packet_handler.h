/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file imu_packet_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

class ImuPacketHandler {
   public:
    using HandlerOutput = sensor_msgs::Imu;
    using HandlerType = std::function<HandlerOutput(const uint8_t*)>;

   public:
    static HandlerType create_handler(const ouster::sensor::sensor_info& info,
                                      const std::string& frame,
                                      bool use_ros_time) {
        const auto& pf = ouster::sensor::get_format(info);
        using Timestamper = std::function<ros::Time(const uint8_t*)>;
        // clang-format off
        auto timestamper = use_ros_time ?
            Timestamper{[](const uint8_t* /*imu_buf*/) {
                return ros::Time::now(); }} :
            Timestamper{[pf](const uint8_t* imu_buf) {
                return ouster_ros::impl::ts_to_ros_time(pf.imu_gyro_ts(imu_buf)); }};
        // clang-format on
        return [&pf, &frame, timestamper](const uint8_t* imu_buf) {
            return ouster_ros::packet_to_imu_msg(pf, timestamper(imu_buf),
                                                 frame, imu_buf);
        };
    }
};