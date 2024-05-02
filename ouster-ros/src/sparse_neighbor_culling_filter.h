/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sparse_neighbor_culling_filter.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class SparseNeighbourCullingFilter {
   private:
    const double neighbor_distance = 50;
    const uint32_t min_neighbors = 3;
    const uint32_t max_range = 2000;

   public:
    SparseNeighbourCullingFilter(const ouster::sensor::sensor_info& info_)
    : info(info_)
    {
    }

   private:
    template <typename T>
    ouster::img_t<T> rotate_x(const ouster::img_t<T>& in, int x) {

        auto m =  in.rows();
        auto n = in.cols();

        // rotate left
        if (x == -1) {
            ouster::img_t<T> shifted(m, n);
            shifted.block(0, 0, m, n-1) = in.block(0, 1, m, n-1);
            shifted.block(0, n-1, m, 1) = in.block(0, 0, m, 1);
            return shifted;
        }

        // rotate right
        if (x == +1) {
            ouster::img_t<T> shifted(m, n);
            shifted.block(0, 1, m, n-1) = in.block(0, 0, m, n-1);
            shifted.block(0, 0, m, 1) = in.block(0, n-1, m, 1);
            return shifted;
        }

        return in;
    }

    template <typename T>
    ouster::img_t<T> rotate_y(const ouster::img_t<T>& in, int y) {

        auto m =  in.rows();
        auto n = in.cols();

        // rotate up
        if (y == -1) {
            ouster::img_t<T> shifted(m, n);
            shifted.block(0, 0, m-1, n) = in.block(1, 0, m-1, n);
            shifted.row(shifted.rows() - 1).setZero();
            return shifted;
        }

        // rotate down
        if (y == +1) {
            ouster::img_t<T> shifted(m, n);
            shifted.block(1, 0, m-1, n) = in.block(0, 0, m-1, n);
            shifted.row(0).setZero();
            return shifted;
        }

        return in;
    }

    void process(ouster::LidarScan& lidar_scan, uint64_t,
                 const rclcpp::Time&) {
        auto range = lidar_scan.field(sensor::ChanField::RANGE);
        long int n = range.cols(); /* azimuth pixels per frame ( sensor firmware dependant) */
        long int m = range.rows(); /* # of sensor beams per frame */

        ouster::img_t<int32_t> neighbors = ouster::img_t<int32_t>::Zero(m, n);
        ouster::img_t<int32_t> des_range =
            ouster::destagger<uint32_t>(range, info.format.pixel_shift_by_row)
                                            .cast<int32_t>();

        /* shift all of our range data up, down, left, and right to search for neighbors */
        for (int x = -1; x <= +1; x++) {
            for (int y = -1; y <= +1; y++) {
                ouster::img_t<int32_t> shifted_x = rotate_x(des_range, x);
                ouster::img_t<int32_t> shifted_xy = rotate_y(shifted_x, y);
                ouster::img_t<int32_t> abs_diff = (shifted_xy - des_range).cwiseAbs();
                neighbors = (shifted_xy != 0 && des_range != 0 && abs_diff < neighbor_distance)
                    .select(neighbors + 1, neighbors);
            }
        }

        range = (neighbors < min_neighbors && range < max_range).select(0, range);
    }

   public:
    static LidarScanProcessor create(const ouster::sensor::sensor_info& info) {
        auto handler =
            std::make_shared<SparseNeighbourCullingFilter>(info);

        return [handler](ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                         const rclcpp::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
     ouster::sensor::sensor_info info;
};

}  // namespace ouster_ros