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
   public:
    SparseNeighbourCullingFilter(const ouster::sensor::sensor_info& info)
    {
    }

   private:
    void sparse_neighbor_culling(Eigen::Ref<ouster::img_t<uint32_t>> range_mat)
    {
        long int n = range_mat.cols(); /* azimuth pixels per frame ( sensor firmware dependant) */
        long int m = range_mat.rows(); /* # of sensor beams per frame */

        ouster::img_t<uint32_t> neighbors_count_mat = ouster::img_t<uint32_t>::Zero(m, n);

        for (int x = -1; x < 2; x++)
        {
            for (int y = -1; y < 2; y++)
            {
                /* shift all of our range data up, down, left, and right to search for neighbors */
                ouster::img_t<uint32_t> shifted_mat = ouster::img_t<uint32_t>::Zero(m, n);
                if(x == -1)
                {
                    /* shift data left. End wraps around */
                    shifted_mat.block(0, 0, m, n-1) = range_mat.block(0, 1, m, n-1);
                    shifted_mat.block(0, n-1, m, 1) = range_mat.block(0, 0, m, 1);
                }
                else if(x == 1)
                {
                    /* shift data right. End wraps around */
                    shifted_mat.block(0, 1, m, n-1) = range_mat.block(0, 0, m, n-1);
                    shifted_mat.block(0, 0, m, 1) = range_mat.block(0, n-1, m, 1);
                }
                if(y == -1)
                {
                    /* shift data down. Top row should be zeros */
                    shifted_mat.block(1, 0, m-1, n) = range_mat.block(0, 0, m-1, n);
                    shifted_mat.block(0, 0, 1, n) = ouster::img_t<uint32_t>::Zero(1, n);
                }
                else if(y == 1)
                {
                    /* shift data up. Bottom row should be zeros */
                    shifted_mat.block(0, 0, m-1, n) = range_mat.block(1, 0, m-1, n);
                    shifted_mat.block(m-1, 0, 1, n) = ouster::img_t<uint32_t>::Zero(1, n);
                }
                
                /* iterate through each point */
                const double neighbor_distance = 0.05;
                for(int i = 0; i < m; i++)
                {
                    for(int j = 0; j < n; j++)
                    {
                        /* If the range value is nonzero, and the shifted matrix is nonzero, and the difference between the two is less than a threshold distance, they are neighbors */
                        neighbors_count_mat(i,j) +=
                            ((range_mat(i,j) != 0) && (shifted_mat(i,j) != 0) && (std::abs<long>(shifted_mat(i,j) - range_mat(i,j)) < neighbor_distance)) ? 1 : 0;
                    }
                }
            }
        }

        const uint32_t min_neighbors = 3;
        const uint32_t max_range = 2000;

        for(int i = 0; i < m; i++)
        {
            for(int j = 0; j < n; j++)
            {
                if(!((neighbors_count_mat(i, j) <= min_neighbors) && (std::abs<long>(range_mat(i,j)) < max_range)))
                {
                    range_mat(i,j) = 0;
                }
            }
        }
    }

    void process(ouster::LidarScan& lidar_scan, uint64_t,
                 const rclcpp::Time&) {
        auto input = lidar_scan.field(sensor::ChanField::RANGE);
        sparse_neighbor_culling(input);
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
};

}  // namespace ouster_ros