/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_ros.cpp
 * @brief A utilty file that contains helper methods
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_eigen/tf2_eigen.h>

#include <chrono>
#include <string>
#include <vector>

namespace sensor = ouster::sensor;

namespace ouster_ros {

bool is_legacy_lidar_profile(const sensor::sensor_info& info) {
    using sensor::UDPProfileLidar;
    return info.format.udp_profile_lidar ==
           UDPProfileLidar::PROFILE_LIDAR_LEGACY;
}

int get_n_returns(const sensor::sensor_info& info) {
    using sensor::UDPProfileLidar;
    return info.format.udp_profile_lidar ==
                   UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
               ? 2
               : 1;
}

size_t get_beams_count(const sensor::sensor_info& info) {
    return info.beam_azimuth_angles.size();
}

std::string topic_for_return(const std::string& base, int idx) {
    return idx == 0 ? base : base + std::to_string(idx + 1);
}

sensor_msgs::Imu packet_to_imu_msg(const ouster::sensor::packet_format& pf,
                                   const ros::Time& timestamp,
                                   const std::string& frame,
                                   const uint8_t* buf) {
    sensor_msgs::Imu m;
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

sensor_msgs::Imu packet_to_imu_msg(const PacketMsg& pm,
                                   const ros::Time& timestamp,
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

}  // namespace impl

template <typename PointT, typename RangeT, typename ReflectivityT,
          typename NearIrT, typename SignalT>
void copy_scan_to_cloud(ouster_ros::Cloud& cloud, const ouster::LidarScan& ls,
                        std::chrono::nanoseconds scan_ts, const PointT& points,
                        const ouster::img_t<RangeT>& range,
                        const ouster::img_t<ReflectivityT>& reflectivity,
                        const ouster::img_t<NearIrT>& near_ir,
                        const ouster::img_t<SignalT>& signal) {
    auto timestamp = ls.timestamp();
    const auto rg = range.data();
    const auto rf = reflectivity.data();
    const auto nr = near_ir.data();
    const auto sg = signal.data();
    const auto t_zero = std::chrono::duration<long int, std::nano>::zero();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for collapse(2)
#endif
    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto col_ts = std::chrono::nanoseconds(timestamp[v]);
            const auto ts = col_ts > scan_ts ? col_ts - scan_ts : t_zero;
            const auto idx = u * ls.w + v;
            const auto xyz = points.row(idx);
            cloud.points[idx] = ouster_ros::Point{
                {static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                 static_cast<float>(xyz(2)), 1.0f},
                static_cast<float>(sg[idx]),
                static_cast<uint32_t>(ts.count()),
                static_cast<uint16_t>(rf[idx]),
                static_cast<uint16_t>(u),
                static_cast<uint16_t>(nr[idx]),
                static_cast<uint32_t>(rg[idx]),
            };
        }
    }
}

template <typename PointT, typename RangeT, typename ReflectivityT,
          typename NearIrT, typename SignalT>
void copy_scan_to_cloud(ouster_ros::Cloud& cloud, const ouster::LidarScan& ls,
                        uint64_t scan_ts, const PointT& points,
                        const ouster::img_t<RangeT>& range,
                        const ouster::img_t<ReflectivityT>& reflectivity,
                        const ouster::img_t<NearIrT>& near_ir,
                        const ouster::img_t<SignalT>& signal) {
    auto timestamp = ls.timestamp();

    const auto rg = range.data();
    const auto rf = reflectivity.data();
    const auto nr = near_ir.data();
    const auto sg = signal.data();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for collapse(2)
#endif
    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto col_ts = timestamp[v];
            const auto ts = col_ts > scan_ts ? col_ts - scan_ts : 0UL;
            const auto idx = u * ls.w + v;
            const auto xyz = points.row(idx);
            cloud.points[idx] = ouster_ros::Point{
                {static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                 static_cast<float>(xyz(2)), 1.0f},
                static_cast<float>(sg[idx]),
                static_cast<uint32_t>(ts),
                static_cast<uint16_t>(rf[idx]),
                static_cast<uint16_t>(u),
                static_cast<uint16_t>(nr[idx]),
                static_cast<uint32_t>(rg[idx]),
            };
        }
    }
}

template <typename PointT, typename RangeT, typename ReflectivityT,
          typename NearIrT, typename SignalT>
void copy_scan_to_cloud_destaggered(
    ouster_ros::Cloud& cloud, const ouster::LidarScan& ls, uint64_t scan_ts,
    const PointT& points, const ouster::img_t<RangeT>& range,
    const ouster::img_t<ReflectivityT>& reflectivity,
    const ouster::img_t<NearIrT>& near_ir, const ouster::img_t<SignalT>& signal,
    const std::vector<int>& pixel_shift_by_row) {
    auto timestamp = ls.timestamp();
    const auto rg = range.data();
    const auto rf = reflectivity.data();
    const auto nr = near_ir.data();
    const auto sg = signal.data();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for collapse(2)
#endif
    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto v_shift = (v + ls.w - pixel_shift_by_row[u]) % ls.w;
            auto ts = timestamp[v_shift]; ts = ts > scan_ts ? ts - scan_ts : 0UL;
            const auto src_idx = u * ls.w + v_shift;
            const auto tgt_idx = u * ls.w + v;
            const auto xyz = points.row(src_idx);
            cloud.points[tgt_idx] = ouster_ros::Point{
                {static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                 static_cast<float>(xyz(2)), 1.0f},
                static_cast<float>(sg[src_idx]),
                static_cast<uint32_t>(ts),
                static_cast<uint16_t>(rf[src_idx]),
                static_cast<uint16_t>(u),
                static_cast<uint16_t>(nr[src_idx]),
                static_cast<uint32_t>(rg[src_idx]),
            };
        }
    }
}

void scan_to_cloud_f(ouster::PointsF& points,
                     const ouster::PointsF& lut_direction,
                     const ouster::PointsF& lut_offset,
                     std::chrono::nanoseconds scan_ts,
                     const ouster::LidarScan& ls, ouster_ros::Cloud& cloud,
                     int return_index) {
    bool second = (return_index == 1);

    assert(cloud.width == static_cast<std::uint32_t>(ls.w) &&
           cloud.height == static_cast<std::uint32_t>(ls.h) &&
           "point cloud and lidar scan size mismatch");

    // across supported lidar profiles range is always 32-bit
    auto range_channel_field =
        second ? sensor::ChanField::RANGE2 : sensor::ChanField::RANGE;
    ouster::img_t<uint32_t> range = ls.field<uint32_t>(range_channel_field);

    ouster::img_t<uint16_t> reflectivity = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::REFLECTIVITY, second), ls);

    ouster::img_t<uint32_t> signal = impl::get_or_fill_zero<uint32_t>(
        impl::suitable_return(sensor::ChanField::SIGNAL, second), ls);

    ouster::img_t<uint16_t> near_ir = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::NEAR_IR, second), ls);

    ouster::cartesianT(points, range, lut_direction, lut_offset);

    copy_scan_to_cloud(cloud, ls, scan_ts, points, range, reflectivity, near_ir,
                       signal);
}

void scan_to_cloud_f(ouster::PointsF& points,
                     const ouster::PointsF& lut_direction,
                     const ouster::PointsF& lut_offset, uint64_t scan_ts,
                     const ouster::LidarScan& ls, ouster_ros::Cloud& cloud,
                     int return_index) {
    bool second = (return_index == 1);

    assert(cloud.width == static_cast<std::uint32_t>(ls.w) &&
           cloud.height == static_cast<std::uint32_t>(ls.h) &&
           "point cloud and lidar scan size mismatch");

    // across supported lidar profiles range is always 32-bit
    auto range_channel_field =
        second ? sensor::ChanField::RANGE2 : sensor::ChanField::RANGE;
    ouster::img_t<uint32_t> range = ls.field<uint32_t>(range_channel_field);

    ouster::img_t<uint16_t> reflectivity = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::REFLECTIVITY, second), ls);

    ouster::img_t<uint32_t> signal = impl::get_or_fill_zero<uint32_t>(
        impl::suitable_return(sensor::ChanField::SIGNAL, second), ls);

    ouster::img_t<uint16_t> near_ir = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::NEAR_IR, second), ls);

    ouster::cartesianT(points, range, lut_direction, lut_offset);

    copy_scan_to_cloud(cloud, ls, scan_ts, points, range, reflectivity, near_ir,
                       signal);
}

void scan_to_cloud_f_destaggered(ouster_ros::Cloud& cloud,
                                 ouster::PointsF& points,
                                 const ouster::PointsF& lut_direction,
                                 const ouster::PointsF& lut_offset,
                                 uint64_t scan_ts, const ouster::LidarScan& ls,
                                 const std::vector<int>& pixel_shift_by_row,
                                 int return_index) {
    bool second = (return_index == 1);

    assert(cloud.width == static_cast<std::uint32_t>(ls.w) &&
           cloud.height == static_cast<std::uint32_t>(ls.h) &&
           "point cloud and lidar scan size mismatch");

    // across supported lidar profiles range is always 32-bit
    auto range_channel_field =
        second ? sensor::ChanField::RANGE2 : sensor::ChanField::RANGE;
    ouster::img_t<uint32_t> range = ls.field<uint32_t>(range_channel_field);

    ouster::img_t<uint16_t> reflectivity = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::REFLECTIVITY, second), ls);

    ouster::img_t<uint32_t> signal = impl::get_or_fill_zero<uint32_t>(
        impl::suitable_return(sensor::ChanField::SIGNAL, second), ls);

    ouster::img_t<uint16_t> near_ir = impl::get_or_fill_zero<uint16_t>(
        impl::suitable_return(sensor::ChanField::NEAR_IR, second), ls);

    ouster::cartesianT(points, range, lut_direction, lut_offset);

    copy_scan_to_cloud_destaggered(cloud, ls, scan_ts, points, range,
                                   reflectivity, near_ir, signal,
                                   pixel_shift_by_row);
}

sensor_msgs::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud,
                                            const ros::Time& timestamp,
                                            const std::string& frame) {
    sensor_msgs::PointCloud2 msg{};
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp = timestamp;
    return msg;
}

geometry_msgs::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame, ros::Time timestamp) {
    Eigen::Affine3d aff;
    aff.linear() = mat.block<3, 3>(0, 0);
    aff.translation() = mat.block<3, 1>(0, 3) * 1e-3;

    geometry_msgs::TransformStamped msg = tf2::eigenToTransform(aff);
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame;
    msg.child_frame_id = child_frame;

    return msg;
}

// TODO: provide a method that accepts sensor_msgs::LaserScan object
sensor_msgs::LaserScan lidar_scan_to_laser_scan_msg(
    const ouster::LidarScan& ls, const ros::Time& timestamp,
    const std::string& frame, const ouster::sensor::lidar_mode ld_mode,
    const uint16_t ring, const int return_index) {
    sensor_msgs::LaserScan msg;
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
    int idx = ls.w * ring + ls.w;
    for (int i = 0; idx-- > ls.w * ring; ++i) {
        msg.ranges[i] = rg[idx] * ouster::sensor::range_unit;
        msg.intensities[i] = static_cast<float>(sg[idx]);
    }

    return msg;
}

}  // namespace ouster_ros
