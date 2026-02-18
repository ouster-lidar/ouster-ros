/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_ros.cpp
 * @brief A utilty file that contains helper methods
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#if __has_include(<tf2/LinearMath/Transform.hpp>)
#include <tf2/LinearMath/Transform.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#endif


#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <regex>


namespace ouster_ros {

using ouster_sensor_msgs::msg::PacketMsg;
using ouster_sensor_msgs::msg::Telemetry;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::LidarScan;
using ouster::sdk::core::LidarMode;
using ouster::sdk::core::cf_type;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::mat4d;
using ouster::sdk::core::Version;
namespace ChanField = ouster::sdk::core::ChanField;

bool is_legacy_lidar_profile(const SensorInfo& info) {
    using ouster::sdk::core::UDPProfileLidar;
    return info.format.udp_profile_lidar ==
           UDPProfileLidar::PROFILE_LIDAR_LEGACY;
}

size_t get_beams_count(const SensorInfo& info) {
    return info.beam_azimuth_angles.size();
}

std::string topic_for_return(const std::string& base, int idx) {
    return idx == 0 ? base : base + std::to_string(idx + 1);
}

std::vector<sensor_msgs::msg::Imu> packet_to_imu_msgs(
    const ouster::sdk::core::ImuPacket& imu_packet,
    const std::string& frame,
    const rclcpp::Time& timestamp,
    const ouster::sdk::core::SensorInfo& sensor_info) {

    std::vector<sensor_msgs::msg::Imu> msgs;
    Eigen::ArrayX<uint16_t> imu_status = imu_packet.status();

    // count valid measurements:
    int valid_count = imu_status.count();
    std::cout << "valid imu_measuremets: " << valid_count << std::endl;
    msgs.reserve(valid_count);

    Eigen::ArrayX<uint64_t> imu_timestamps = imu_packet.timestamp();
    if (imu_packet.format->imu_measurements_per_packet == 0) {  // Handle the LEGACY IMU profile (it would be better if the imu_packet handles this internally).
        imu_timestamps[0] = imu_packet.format->imu_gyro_ts(imu_packet.buf.data());
    } else  if (imu_timestamps[0] == 0) {   // HANDLE the case when the first imu_timestamp is unknown.
        int total_frame_imu_measurements = imu_packet.format->imu_measurements_per_packet * imu_packet.format->imu_packets_per_frame;
        double frame_ts = 1e9 / sensor_info.format.fps;
        double imu_measurement_interval = frame_ts / total_frame_imu_measurements;
        for (int i = 0; i < imu_timestamps.size(); ++i) {
            imu_timestamps[i] = imu_timestamps[0] + static_cast<uint64_t>(i * imu_measurement_interval);
        }
    }

    Eigen::ArrayX3f accel = imu_packet.accel();
    Eigen::ArrayX3f gyro = imu_packet.gyro();

    sensor_msgs::msg::Imu m;
    m.orientation_covariance = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
    m.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
    m.angular_velocity_covariance = {6e-4, 0, 0, 0, 6e-4, 0, 0, 0, 6e-4};
    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 1;

    for (int i = 0; i < imu_status.size(); ++i) {
        if ((imu_status[i] & 0x1) == 0) {
            continue;
        }

        m.header.frame_id = frame;
        auto ts_offset = std::chrono::nanoseconds(imu_timestamps[i] - imu_timestamps[0]);
        m.header.stamp = timestamp + rclcpp::Duration(ts_offset);
        m.linear_acceleration.x = accel(i, 0);
        m.linear_acceleration.y = accel(i, 1);
        m.linear_acceleration.z = accel(i, 2);
        m.angular_velocity.x = gyro(i, 0);
        m.angular_velocity.y = gyro(i, 1);
        m.angular_velocity.z = gyro(i, 2);

        msgs.push_back(m);
    }

    return msgs;
}

namespace impl {
std::string scan_return(const std::string& field, bool second) {
    if (field == ChanField::RANGE || field == ChanField::RANGE2) {
        return second ? ChanField::RANGE2 : ChanField::RANGE;
    } else if (field == ChanField::SIGNAL || field == ChanField::SIGNAL2) {
        return second ? ChanField::SIGNAL2 : ChanField::SIGNAL;
    } else if (field == ChanField::REFLECTIVITY || field == ChanField::REFLECTIVITY2) {
        return second ? ChanField::REFLECTIVITY2 : ChanField::REFLECTIVITY;
    } else if (field == ChanField::NEAR_IR) {
        return ChanField::NEAR_IR;
    } else {
        throw std::runtime_error("Unreachable");
    }
}

// TODO: move to a separate file
std::set<std::string> parse_tokens(const std::string& input, char delim) {

    std::set<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (getline(ss, token, delim)) {
        // Remove leading and trailing whitespaces from the token
        size_t start = token.find_first_not_of(" ");
        size_t end = token.find_last_not_of(" ");
        token = token.substr(start, end - start + 1);
        if (!token.empty()) tokens.insert(token);
    }

    return tokens;
}

Version parse_version(const std::string& fw_rev) {
    auto rgx = std::regex(R"(v(\d+).(\d+)\.(\d+))");
    std::smatch matches;
    std::regex_search(fw_rev, matches, rgx);

    if (matches.size() < 4) return ouster::sdk::core::INVALID_VERSION;

    try {
        return Version{static_cast<uint16_t>(stoul(matches[1])),
                    static_cast<uint16_t>(stoul(matches[2])),
                    static_cast<uint16_t>(stoul(matches[3]))};
    } catch (const std::exception&) {
        return ouster::sdk::core::INVALID_VERSION;
    }
}

void warn_mask_resized(int image_cols, int image_rows,
                       int scan_height, int scan_width) {
    auto logger = rclcpp::get_logger("ouster_ros");
    RCLCPP_WARN_STREAM(logger, "Mask image has size (" << image_cols << "x" << image_rows << ")"
                       << " but incoming scans has size (" << scan_height << "x" << scan_width << ")."
                       << " Resizing mask to match the scans size.");    
}

}  // namespace impl

geometry_msgs::msg::TransformStamped transform_to_tf_msg(
    const mat4d& mat, const std::string& frame,
    const std::string& child_frame, rclcpp::Time timestamp) {
    Eigen::Affine3d aff;
    aff.linear() = mat.block<3, 3>(0, 0);
    aff.translation() = mat.block<3, 1>(0, 3) * 1e-3;

    geometry_msgs::msg::TransformStamped msg = tf2::eigenToTransform(aff);
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame;
    msg.child_frame_id = child_frame;

    return msg;
}

// TODO: provide a method that accepts sensor_msgs::msg::LaserScan object
sensor_msgs::msg::LaserScan lidar_scan_to_laser_scan_msg(
    const LidarScan& ls, const rclcpp::Time& timestamp,
    const std::string& frame, const LidarMode ld_mode,
    const uint16_t ring, const std::vector<int>& pixel_shift_by_row,
    const int return_index) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame;
    msg.angle_min = -M_PI;   // TODO: configure to match the actual scan window
    msg.angle_max = M_PI;    // TODO: configure to match the actual scan window
    msg.range_min = 0.1f;    // TODO: fill per product type
    msg.range_max = 120.0f;  // TODO: fill per product type

    const auto scan_width = ouster::sdk::core::n_cols_of_lidar_mode(ld_mode);
    const auto scan_frequency = ouster::sdk::core::frequency_of_lidar_mode(ld_mode);
    msg.scan_time = 1.0f / scan_frequency;
    msg.time_increment = 1.0f / (scan_width * scan_frequency);
    msg.angle_increment = 2 * M_PI / scan_width;

    auto which_range = return_index == 0 ? ChanField::RANGE
                                         : ChanField::RANGE2;
    ouster::sdk::core::img_t<uint32_t> range = ls.field<uint32_t>(which_range);
    auto which_signal = return_index == 0 ? ChanField::SIGNAL
                                          : ChanField::SIGNAL2;
    ouster::sdk::core::img_t<uint32_t> signal =
        impl::get_or_fill_zero<uint32_t>(which_signal, ls);
    const auto rg = range.data();
    const auto sg = signal.data();
    msg.ranges.resize(ls.w);
    msg.intensities.resize(ls.w);

    uint16_t u = ring;
    for (int v =  0; v < static_cast<int>(ls.w); ++v) {
        auto v_shift = (v + ls.w - pixel_shift_by_row[u] + ls.w / 2) % ls.w;
        auto src_idx = u * ls.w + v_shift;
        auto tgt_idx = ls.w - 1 - v;
        msg.ranges[tgt_idx] = rg[src_idx] * ouster::sdk::core::RANGE_UNIT;
        msg.intensities[tgt_idx] = static_cast<float>(sg[src_idx]);
    }

    return msg;
}

Telemetry lidar_packet_to_telemetry_msg(
    const LidarPacket& lidar_packet, const rclcpp::Time& timestamp,
    const PacketFormat& pf) {
    Telemetry telemetry;
    telemetry.header.stamp = timestamp;
    telemetry.countdown_thermal_shutdown = pf.countdown_thermal_shutdown(lidar_packet.buf.data());
    telemetry.countdown_shot_limiting = pf.countdown_shot_limiting(lidar_packet.buf.data());
    telemetry.thermal_shutdown = pf.thermal_shutdown(lidar_packet.buf.data());
    telemetry.shot_limiting = pf.shot_limiting(lidar_packet.buf.data());
    return telemetry;
}

}  // namespace ouster_ros
