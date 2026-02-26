/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file laser_scan_processor.h
 * @brief Takes in a lidar scan object and produces a LaserScan message
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
    LaserScanProcessor(const ouster::sdk::core::SensorInfo& info,
                       const std::string& frame_id, uint16_t ring,
                       PostProcessingFn func)
        : frame(frame_id),
          ld_mode(info.config.lidar_mode.value()),
          ring_(ring),
          pixel_shift_by_row(info.format.pixel_shift_by_row),
          scan_msgs(info.num_returns()),
          post_processing_fn(func) {
        for (size_t i = 0; i < scan_msgs.size(); ++i)
            scan_msgs[i] = std::make_shared<sensor_msgs::LaserScan>();

        const auto fw = impl::parse_version(info.fw_rev);
        if (fw.major == 2 && fw.minor < 4) {
            std::transform(pixel_shift_by_row.begin(),
                           pixel_shift_by_row.end(),
                           pixel_shift_by_row.begin(),
                           [](auto c) { return c - 31; });
        }
    }

   private:
    void process(const ouster::sdk::core::LidarScan& lidar_scan, uint64_t,
                 const ros::Time& msg_ts) {
        for (size_t i = 0; i < scan_msgs.size(); ++i) {
            *scan_msgs[i] =
                lidar_scan_to_laser_scan_msg(lidar_scan, msg_ts, frame, ld_mode,
                                             ring_, pixel_shift_by_row, i);
        }

        if (post_processing_fn) post_processing_fn(scan_msgs);
    }

   public:
    static LidarScanProcessor create(const ouster::sdk::core::SensorInfo& info,
                                     const std::string& frame, uint16_t ring,
                                     PostProcessingFn func) {
        auto handler =
            std::make_shared<LaserScanProcessor>(info, frame, ring, func);

        return [handler](const ouster::sdk::core::LidarScan& lidar_scan, uint64_t scan_ts,
                         const ros::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    std::string frame;
    ouster::sdk::core::LidarMode ld_mode;
    uint16_t ring_;
    std::vector<int> pixel_shift_by_row;
    OutputType scan_msgs;
    PostProcessingFn post_processing_fn;
};

}  // namespace ouster_ros