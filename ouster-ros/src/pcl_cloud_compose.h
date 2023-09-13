/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file pcl_cloud_compose.h
 * @brief ...
 */

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "ouster_ros/os_point.h"


namespace ouster_ros {

namespace sensor = ouster::sensor;

template <class T>
using Cloud = pcl::PointCloud<T>;

// TODO: describe or hide
template <typename PointT, typename RangeT, typename ReflectivityT,
          typename NearIrT, typename SignalT>
void copy_scan_to_cloud_destaggered(
    ouster_ros::Cloud<Point>& cloud, const ouster::LidarScan& ls, uint64_t scan_ts,
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

// TODO: describe or hide
template <typename PointT, typename RangeT, typename ReflectivityT,
          typename NearIrT, typename SignalT>
void copy_scan_to_cloud_destaggered(
    ouster_ros::Cloud<PointXYZIR>& cloud, const ouster::LidarScan& ls, uint64_t /*scan_ts*/,
    const PointT& points, const ouster::img_t<RangeT>& /*range*/,
    const ouster::img_t<ReflectivityT>& /*reflectivity*/,
    const ouster::img_t<NearIrT>& /*near_ir*/, const ouster::img_t<SignalT>& signal,
    const std::vector<int>& pixel_shift_by_row) {

    const auto sg = signal.data();

#ifdef __OUSTER_UTILIZE_OPENMP__
#pragma omp parallel for collapse(2)
#endif
    for (auto u = 0; u < ls.h; u++) {
        for (auto v = 0; v < ls.w; v++) {
            const auto v_shift = (v + ls.w - pixel_shift_by_row[u]) % ls.w;
            const auto src_idx = u * ls.w + v_shift;
            const auto tgt_idx = u * ls.w + v;
            const auto xyz = points.row(src_idx);
            cloud.points[tgt_idx] = ouster_ros::PointXYZIR{
                {static_cast<float>(xyz(0)), static_cast<float>(xyz(1)),
                 static_cast<float>(xyz(2)), 1.0f},
                static_cast<float>(sg[src_idx]),
                static_cast<uint16_t>(u)
            };
        }
    }
}

/**
 * Populate a destaggered PCL point cloud from a LidarScan
 * @param[out] cloud output pcl pointcloud to populate
 * @param[in, out] points The points parameters is used to store the results of
 * the cartesian product before it gets packed into the cloud object.
 * @param[in] lut_direction the direction of the xyz lut (with single precision)
 * @param[in] lut_offset the offset of the xyz lut (with single precision)
 * @param[in] scan_ts scan start used to caluclate relative timestamps for
 * points
 * @param[in] lidar_scan input lidar data
 * @param[in] pixel_shift_by_row pixel shifts by row
 * @param[in] return_index index of return desired starting at 0
 */
template <typename T>
void scan_to_cloud_f_destaggered(ouster_ros::Cloud<T>& cloud,
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

}   // namespace ouster_ros