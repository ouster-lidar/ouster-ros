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

#include <tf2/LinearMath/Transform.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <string>
#include <vector>
#include <regex>


namespace ouster_ros {

namespace sensor = ouster::sensor;
using namespace ouster::util;
using ouster_sensor_msgs::msg::PacketMsg;


bool is_legacy_lidar_profile(const sensor::sensor_info& info) {
    using sensor::UDPProfileLidar;
    return info.format.udp_profile_lidar ==
           UDPProfileLidar::PROFILE_LIDAR_LEGACY;
}

int get_n_returns(const sensor::sensor_info& info) {
    using sensor::UDPProfileLidar;
    if (info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL ||
        info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_FUSA_RNG15_RFL8_NIR8_DUAL)
        return 2;

    return 1;
}

size_t get_beams_count(const sensor::sensor_info& info) {
    return info.beam_azimuth_angles.size();
}

std::string topic_for_return(const std::string& base, int idx) {
    return idx == 0 ? base : base + std::to_string(idx + 1);
}

sensor_msgs::msg::Imu packet_to_imu_msg(const ouster::sensor::packet_format& pf,
                                        const rclcpp::Time& timestamp,
                                        const std::string& frame,
                                        const uint8_t* buf) {
    sensor_msgs::msg::Imu m;
    m.header.stamp = timestamp;
    m.header.frame_id = frame;

    m.orientation.x = 0;
    m.orientation.y = 0;
    m.orientation.z = 0;
    m.orientation.w = 1;

    const double standard_g = 9.80665;
    m.linear_acceleration.x = pf.imu_la_x(buf) * standard_g;
    m.linear_acceleration.y = pf.imu_la_y(buf) * standard_g;
    m.linear_acceleration.z = pf.imu_la_z(buf) * standard_g;

    m.angular_velocity.x = pf.imu_av_x(buf) * M_PI / 180.0;
    m.angular_velocity.y = pf.imu_av_y(buf) * M_PI / 180.0;
    m.angular_velocity.z = pf.imu_av_z(buf) * M_PI / 180.0;

    for (int i = 0; i < 9; i++) {
        m.orientation_covariance[i] = -1;
        m.angular_velocity_covariance[i] = 0;
        m.linear_acceleration_covariance[i] = 0;
    }

    for (int i = 0; i < 9; i += 4) {
        m.linear_acceleration_covariance[i] = 0.01;
        m.angular_velocity_covariance[i] = 6e-4;
    }

    return m;
}

sensor_msgs::msg::Imu packet_to_imu_msg(const PacketMsg& pm,
                                        const rclcpp::Time& timestamp,
                                        const std::string& frame,
                                        const sensor::packet_format& pf) {
    return packet_to_imu_msg(pf, timestamp, frame, pm.buf.data());
}

namespace impl {
sensor::ChanField suitable_return(sensor::ChanField input_field, bool second) {
    switch (input_field) {
        case sensor::ChanField::RANGE:
        case sensor::ChanField::RANGE2:
            return second ? sensor::ChanField::RANGE2
                          : sensor::ChanField::RANGE;
        case sensor::ChanField::SIGNAL:
        case sensor::ChanField::SIGNAL2:
            return second ? sensor::ChanField::SIGNAL2
                          : sensor::ChanField::SIGNAL;
        case sensor::ChanField::REFLECTIVITY:
        case sensor::ChanField::REFLECTIVITY2:
            return second ? sensor::ChanField::REFLECTIVITY2
                          : sensor::ChanField::REFLECTIVITY;
        case sensor::ChanField::NEAR_IR:
            return sensor::ChanField::NEAR_IR;
        default:
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

version parse_version(const std::string& fw_rev) {
    auto rgx = std::regex(R"(v(\d+).(\d+)\.(\d+))");
    std::smatch matches;
    std::regex_search(fw_rev, matches, rgx);

    if (matches.size() < 4) return invalid_version;

    try {
        return version{static_cast<uint16_t>(stoul(matches[1])),
                    static_cast<uint16_t>(stoul(matches[2])),
                    static_cast<uint16_t>(stoul(matches[3]))};
    } catch (const std::exception&) {
        return invalid_version;
    }
}

}  // namespace impl

geometry_msgs::msg::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
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
    const ouster::LidarScan& ls, const rclcpp::Time& timestamp,
    const std::string& frame, const ouster::sensor::lidar_mode ld_mode,
    const uint16_t ring, const std::vector<int>& pixel_shift_by_row,
    const int return_index) {
    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame;
    msg.angle_min = -M_PI;   // TODO: configure to match the actual scan window
    msg.angle_max = M_PI;    // TODO: configure to match the actual scan window
    msg.range_min = 0.1f;    // TODO: fill per product type
    msg.range_max = 120.0f;  // TODO: fill per product type

    const auto scan_width = sensor::n_cols_of_lidar_mode(ld_mode);
    const auto scan_frequency = sensor::frequency_of_lidar_mode(ld_mode);
    msg.scan_time = 1.0f / scan_frequency;
    msg.time_increment = 1.0f / (scan_width * scan_frequency);
    msg.angle_increment = 2 * M_PI / scan_width;

    auto which_range = return_index == 0 ? sensor::ChanField::RANGE
                                         : sensor::ChanField::RANGE2;
    ouster::img_t<uint32_t> range = ls.field<uint32_t>(which_range);
    auto which_signal = return_index == 0 ? sensor::ChanField::SIGNAL
                                          : sensor::ChanField::SIGNAL2;
    ouster::img_t<uint32_t> signal =
        impl::get_or_fill_zero<uint32_t>(which_signal, ls);
    const auto rg = range.data();
    const auto sg = signal.data();
    msg.ranges.resize(ls.w);
    msg.intensities.resize(ls.w);

    uint16_t u = ring;
    for (auto v =  0; v < ls.w; ++v) {
        auto v_shift = (v + ls.w - pixel_shift_by_row[u] + ls.w / 2) % ls.w;
        auto src_idx = u * ls.w + v_shift;
        auto tgt_idx = ls.w - 1 - v;
        msg.ranges[tgt_idx] = rg[src_idx] * ouster::sensor::range_unit;
        msg.intensities[tgt_idx] = static_cast<float>(sg[src_idx]);
    }

    return msg;
}

}  // namespace ouster_ros
