/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_cloud_producer.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

namespace ouster_ros {

class LaserScanProcessor {
   public:
    using OutputType = std::vector<std::shared_ptr<sensor_msgs::LaserScan>>;
    using PostProcessingFn = std::function<void(OutputType)>;

   public:
    LaserScanProcessor(const ouster::sensor::sensor_info& info,
                       const std::string& frame_id, uint16_t ring,
                       PostProcessingFn func)
        : frame(frame_id),
          ld_mode(info.mode),
          ring_(ring),
          scan_msgs(get_n_returns(info),
                    std::make_shared<sensor_msgs::LaserScan>()),
          post_processing_fn(func) {}

   private:
    void process(const ouster::LidarScan& lidar_scan, uint64_t,
                 const ros::Time& msg_ts) {
        for (int i = 0; i < static_cast<int>(scan_msgs.size()); ++i) {
            *scan_msgs[i] = lidar_scan_to_laser_scan_msg(
                lidar_scan, msg_ts, frame, ld_mode, ring_, i);
        }

        if (post_processing_fn) post_processing_fn(scan_msgs);
    }

   public:
    static LidarScanProcessor create(const ouster::sensor::sensor_info& info,
                                     const std::string& frame, uint16_t ring,
                                     PostProcessingFn func) {
        auto handler =
            std::make_shared<LaserScanProcessor>(info, frame, ring, func);

        return [handler](const ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                         const ros::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    std::string frame;
    sensor::lidar_mode ld_mode;
    uint16_t ring_;
    OutputType scan_msgs;
    PostProcessingFn post_processing_fn;
};

}  // namespace ouster_ros