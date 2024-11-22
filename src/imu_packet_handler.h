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


#include "imu_model.h"
namespace ouster_ros {

class ImuPacketHandler {
   public:
    using HandlerOutput = sensor_msgs::Imu;
    using HandlerType = std::function<HandlerOutput(const sensor::ImuPacket&)>;

   public:
    static HandlerType create_handler(const sensor::sensor_info& info,
                                      const std::string& frame,
                                      const std::string& timestamp_mode,
                                      int64_t ptp_utc_tai_offset,
                                      bool estimate_orientation) {
        const auto& pf = sensor::get_format(info);
        using Timestamper = std::function<ros::Time(const sensor::ImuPacket&)>;
        Timestamper timestamper;
        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            timestamper = Timestamper{
                [](const sensor::ImuPacket& imu_packet) {
                    return impl::ts_to_ros_time(imu_packet.host_timestamp);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            timestamper = Timestamper{
                [pf, ptp_utc_tai_offset](const sensor::ImuPacket& imu_packet) {
                    auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                    ts = impl::ts_safe_offset_add(ts, ptp_utc_tai_offset);
                    return impl::ts_to_ros_time(ts);
                }};
        } else {
            timestamper = Timestamper{
                [pf](const sensor::ImuPacket& imu_packet) {
                    auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                    return impl::ts_to_ros_time(ts);
                }};
        }

        std::shared_ptr<ImuModel> imu_model;
        if (estimate_orientation) {
            imu_model = std::make_shared<SimpleImuModel>();
            imu_model->set_initial_state(Eigen::Vector3d(0.0, 0.0, 0.0));
        }

        return [&pf, &frame, timestamper, estimate_orientation, imu_model](const sensor::ImuPacket& imu_packet) {
            sensor_msgs::Imu msg = packet_to_imu_msg(pf, timestamper(imu_packet),
                                                     frame, imu_packet.buf.data());
            if (estimate_orientation) {
                auto ts = pf.imu_gyro_ts(imu_packet.buf.data());
                Eigen::Vector3d la(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
                Eigen::Vector3d av(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
                Eigen::Quaternion q = imu_model->update(ts, la, av);
                msg.orientation.x = q.x();
                msg.orientation.y = q.y();
                msg.orientation.z = q.z();
                msg.orientation.w = q.w();
            }
            return msg;
        };
    }
};

}  // namespace ouster_ros