/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_cloud_producer.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include "point_cloud_compose.h"
#include "lidar_packet_handler.h"
#include "impl/cartesian.h"

namespace ouster_ros {

// Moved out of PointCloudProcessor to avoid type templatization
using PointCloudProcessor_OutputType =
    std::vector<std::shared_ptr<sensor_msgs::msg::PointCloud2>>;
using PointCloudProcessor_PostProcessingFn = std::function<void(PointCloudProcessor_OutputType)>;


template <class PointT>
class PointCloudProcessor {
   public:
    using ScanToCloudFn = std::function<void(ouster_ros::Cloud<PointT>& cloud,
                                        const ouster::PointsF& points,
                                        uint64_t scan_ts, const ouster::LidarScan& ls,
                                        const std::vector<int>& pixel_shift_by_row,
                                        int return_index)>;

   public:
    PointCloudProcessor(const ouster::sensor::sensor_info& info,
                        const std::string& frame_id,
                        bool apply_lidar_to_sensor_transform,
                        ScanToCloudFn scan_to_cloud_fn_,
                        PointCloudProcessor_PostProcessingFn post_processing_fn_)
        : frame(frame_id),
          pixel_shift_by_row(info.format.pixel_shift_by_row),
          cloud{info.format.columns_per_frame, info.format.pixels_per_column},
          pc_msgs(get_n_returns(info)),
          scan_to_cloud_fn(scan_to_cloud_fn_),
          post_processing_fn(post_processing_fn_) {
        for (size_t i = 0; i < pc_msgs.size(); ++i)
            pc_msgs[i] = std::make_shared<sensor_msgs::msg::PointCloud2>();
        ouster::mat4d additional_transform =
            apply_lidar_to_sensor_transform ? info.lidar_to_sensor_transform
                                            : ouster::mat4d::Identity();
        auto xyz_lut = ouster::make_xyz_lut(
            info.format.columns_per_frame, info.format.pixels_per_column,
            ouster::sensor::range_unit, info.beam_to_lidar_transform,
            additional_transform, info.beam_azimuth_angles,
            info.beam_altitude_angles);
        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        // cloud xyz coordinates using double precision (for the time being).
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(lut_direction.rows(), lut_offset.cols());
    }

   private:
    template <typename T>
    void pcl_toROSMsg(const ouster_ros::Cloud<T>& pcl_cloud,
                      sensor_msgs::msg::PointCloud2& cloud) {
        // TODO: remove the staging step in the future
        pcl::toPCLPointCloud2(pcl_cloud, staging_pcl_pc2);
        pcl_conversions::moveFromPCL(staging_pcl_pc2, cloud);
    }

    void process(const ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                 const rclcpp::Time& msg_ts) {
        for (int i = 0; i < static_cast<int>(pc_msgs.size()); ++i) {
            auto range_channel = static_cast<sensor::ChanField>(sensor::ChanField::RANGE + i);
            auto range = lidar_scan.field<uint32_t>(range_channel);
            ouster::cartesianT(points, range, lut_direction, lut_offset,
                               std::numeric_limits<float>::quiet_NaN());

            scan_to_cloud_fn(cloud, points, scan_ts, lidar_scan,
                                        pixel_shift_by_row, i);

            pcl_toROSMsg(cloud, *pc_msgs[i]);
            pc_msgs[i]->header.stamp = msg_ts;
            pc_msgs[i]->header.frame_id = frame;
        }

        if (post_processing_fn) post_processing_fn(pc_msgs);
    }

   public:
    static LidarScanProcessor create(const ouster::sensor::sensor_info& info,
                                     const std::string& frame,
                                     bool apply_lidar_to_sensor_transform,
                                     ScanToCloudFn scan_to_cloud_fn_,
                                     PointCloudProcessor_PostProcessingFn post_processing_fn) {
        auto handler = std::make_shared<PointCloudProcessor>(
            info, frame, apply_lidar_to_sensor_transform, scan_to_cloud_fn_, post_processing_fn);

        return [handler](const ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                         const rclcpp::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    // a buffer used for staging during the conversion
    // from a PCL point cloud to a ros point cloud message
    pcl::PCLPointCloud2 staging_pcl_pc2;

    std::string frame;

    ouster::PointsF lut_direction;
    ouster::PointsF lut_offset;
    ouster::PointsF points;
    std::vector<int> pixel_shift_by_row;
    ouster_ros::Cloud<PointT> cloud;
    PointCloudProcessor_OutputType pc_msgs;
    ScanToCloudFn scan_to_cloud_fn;
    PointCloudProcessor_PostProcessingFn post_processing_fn;
};

}  // namespace ouster_ros