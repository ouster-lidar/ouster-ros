/**
 * Copyright (c) 2018-2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file slam_processor.h
 * @brief takes in a lidar scan object, registers it with the ouster-sdk
 * SLAMEngine and produces odometry, a trajectory path, and a downsampled,
 * accumulated map point cloud.
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h",
// the header file needs to be the first include due to PCL_NO_PRECOMPILE
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <ouster/lidar_scan_set.h>
#include <ouster/slam_engine.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "impl/cartesian.h"

namespace ouster_ros {

class SlamProcessor {
   public:
    struct OutputType {
        std::shared_ptr<nav_msgs::msg::Odometry> odom;
        std::shared_ptr<nav_msgs::msg::Path> trajectory;
        // null unless the accumulated map was refreshed on this update
        std::shared_ptr<sensor_msgs::msg::PointCloud2> map;
    };
    using PostProcessingFn = std::function<void(OutputType)>;

   public:
    SlamProcessor(const ouster::sdk::core::SensorInfo& info,
                 const std::string& odom_frame_id,
                 const std::string& child_frame_id, double min_range,
                 double max_range, double voxel_size,
                 const std::string& deskew_method, double map_voxel_size,
                 int map_publish_every_n, PostProcessingFn func)
        : odom_frame(odom_frame_id),
          child_frame(child_frame_id),
          min_range_mm(impl::ulround(min_range * 1000)),
          max_range_mm(impl::ulround(max_range * 1000)),
          map_publish_every_n_(std::max(1, map_publish_every_n)),
          post_processing_fn(func),
          sensor_info_(info) {
        slam_config.backend = "kiss";
        slam_config.deskew_method = deskew_method;
        slam_config.min_range = min_range;
        slam_config.max_range = max_range;
        slam_config.voxel_size = voxel_size;
        slam_engine = make_slam_engine();

        // The SlamEngine (kiss backend) always registers scans and reports
        // poses in the sensor's own extrinsics-applied frame (it builds its
        // internal LUT via ouster::sdk::core::XYZLut(info, /*use_extrinsics=*/
        // true), which unconditionally applies lidar_to_sensor_transform).
        // Build our own LUT the exact same way so map points and poses line
        // up in the same frame.
        auto xyz_lut = ouster::sdk::core::make_xyz_lut(info, true);
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points_buffer = ouster::sdk::core::PointCloudXYZf(
            static_cast<int>(info.format.pixels_per_column *
                             info.format.columns_per_frame),
            3);

        map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        voxel_filter.setLeafSize(static_cast<float>(map_voxel_size),
                                 static_cast<float>(map_voxel_size),
                                 static_cast<float>(map_voxel_size));

        odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.frame_id = odom_frame;
        odom_msg->child_frame_id = child_frame;

        trajectory_msg = std::make_shared<nav_msgs::msg::Path>();
        trajectory_msg->header.frame_id = odom_frame;

        map_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    }

   private:
    std::unique_ptr<ouster::sdk::mapping::SlamEngine> make_slam_engine() {
        return std::make_unique<ouster::sdk::mapping::SlamEngine>(
            std::vector<std::shared_ptr<ouster::sdk::core::SensorInfo>>{
                std::make_shared<ouster::sdk::core::SensorInfo>(
                    sensor_info_)},
            slam_config);
    }

    // A rosbag replay looping (or any other source of sensor time jumping
    // backwards) means the next scan no longer follows on from whatever the
    // SlamEngine and our own accumulated map/trajectory currently think is
    // the last pose -- registering it against that stale state would at
    // best corrupt the map, and has been observed to throw out of the
    // engine's pose interpolation/deskewing outright. Start a fresh SLAM
    // session instead.
    void reset_for_new_session() {
        RCLCPP_WARN(rclcpp::get_logger("os_cloud"),
                    "SLAM: detected scan timestamps moving backwards (bag "
                    "loop?); resetting odometry, trajectory and the "
                    "accumulated map for a new session.");

        slam_engine = make_slam_engine();
        map_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        trajectory_msg->poses.clear();
        have_prev_pose = false;
        scan_count = 0;
    }

    void process(const ouster::sdk::core::LidarScan& lidar_scan, uint64_t,
                const rclcpp::Time&) {
        const int raw_col = lidar_scan.get_last_valid_column();
        if (raw_col >= 0) {
            const uint64_t raw_ts = lidar_scan.timestamp()[raw_col];
            if (have_last_scan_ts && raw_ts < last_scan_ts_ns) {
                reset_for_new_session();
            }
            last_scan_ts_ns = raw_ts;
            have_last_scan_ts = true;
        }

        // SlamEngine::update() mutates the scan in place (writes the
        // estimated per-column poses back into it); copy it first so the
        // original scan shared with the other processors (PCL/SCAN/...) is
        // left untouched.
        auto scan =
            std::make_shared<ouster::sdk::core::LidarScan>(lidar_scan);
        ouster::sdk::core::LidarScanSet scan_set({scan});
        slam_engine->update(scan_set);

        const auto& updated = scan_set[0];
        const int col = updated->get_last_valid_column();
        if (col < 0) return;

        const ouster::sdk::core::Matrix4dR pose =
            updated->get_column_pose(col);
        const uint64_t pose_ts = updated->timestamp()[col];
        const rclcpp::Time stamp(static_cast<int64_t>(pose_ts), RCL_ROS_TIME);

        const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
        const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
        const Eigen::Quaterniond q(rotation);

        odom_msg->header.stamp = stamp;
        odom_msg->pose.pose.position.x = translation.x();
        odom_msg->pose.pose.position.y = translation.y();
        odom_msg->pose.pose.position.z = translation.z();
        odom_msg->pose.pose.orientation.x = q.x();
        odom_msg->pose.pose.orientation.y = q.y();
        odom_msg->pose.pose.orientation.z = q.z();
        odom_msg->pose.pose.orientation.w = q.w();

        if (have_prev_pose) {
            const double dt = (stamp - prev_stamp).seconds();
            if (dt > 1e-6) {
                // body-frame linear velocity approximated from consecutive
                // SLAM poses; no independent velocity estimate is available.
                const Eigen::Vector3d body_v =
                    prev_rotation.transpose() *
                    (translation - prev_translation) / dt;
                odom_msg->twist.twist.linear.x = body_v.x();
                odom_msg->twist.twist.linear.y = body_v.y();
                odom_msg->twist.twist.linear.z = body_v.z();
            }
        }
        prev_translation = translation;
        prev_rotation = rotation;
        prev_stamp = stamp;
        have_prev_pose = true;

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg->header;
        pose_stamped.pose = odom_msg->pose.pose;
        trajectory_msg->header.stamp = stamp;
        trajectory_msg->poses.push_back(pose_stamped);

        OutputType output;
        output.odom = odom_msg;
        output.trajectory = trajectory_msg;

        if (++scan_count % static_cast<uint64_t>(map_publish_every_n_) == 0) {
            accumulate_map(*updated, pose);
            pcl::toPCLPointCloud2(*map_cloud, staging_pcl_pc2);
            pcl_conversions::moveFromPCL(staging_pcl_pc2, *map_msg);
            map_msg->header.frame_id = odom_frame;
            map_msg->header.stamp = stamp;
            output.map = map_msg;
        }

        if (post_processing_fn) post_processing_fn(output);
    }

    void accumulate_map(const ouster::sdk::core::LidarScan& scan,
                        const ouster::sdk::core::Matrix4dR& pose) {
        auto range = scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        ouster::cartesianT(points_buffer, range, lut_direction, lut_offset,
                           min_range_mm, max_range_mm,
                           std::numeric_limits<float>::quiet_NaN());

        const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
        const Eigen::Vector3d translation = pose.block<3, 1>(0, 3);

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(
            new pcl::PointCloud<pcl::PointXYZ>());
        scan_cloud->reserve(static_cast<size_t>(points_buffer.rows()));
        for (int i = 0; i < points_buffer.rows(); ++i) {
            const float x = points_buffer(i, 0);
            const float y = points_buffer(i, 1);
            const float z = points_buffer(i, 2);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            const Eigen::Vector3d world_pt =
                rotation * Eigen::Vector3d(x, y, z) + translation;
            scan_cloud->emplace_back(static_cast<float>(world_pt.x()),
                                     static_cast<float>(world_pt.y()),
                                     static_cast<float>(world_pt.z()));
        }

        *map_cloud += *scan_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
            new pcl::PointCloud<pcl::PointXYZ>());
        voxel_filter.setInputCloud(map_cloud);
        voxel_filter.filter(*filtered);
        map_cloud = filtered;
    }

   public:
    static LidarScanProcessor create(
        const ouster::sdk::core::SensorInfo& info,
        const std::string& odom_frame_id, const std::string& child_frame_id,
        double min_range, double max_range, double voxel_size,
        const std::string& deskew_method, double map_voxel_size,
        int map_publish_every_n, PostProcessingFn func) {
        auto handler = std::make_shared<SlamProcessor>(
            info, odom_frame_id, child_frame_id, min_range, max_range,
            voxel_size, deskew_method, map_voxel_size, map_publish_every_n,
            func);

        return [handler](const ouster::sdk::core::LidarScan& lidar_scan,
                         uint64_t scan_ts, const rclcpp::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    std::string odom_frame;
    std::string child_frame;
    uint32_t min_range_mm;
    uint32_t max_range_mm;
    int map_publish_every_n_;
    uint64_t scan_count{0};
    PostProcessingFn post_processing_fn;

    ouster::sdk::core::SensorInfo sensor_info_;
    ouster::sdk::mapping::SlamConfig slam_config;
    std::unique_ptr<ouster::sdk::mapping::SlamEngine> slam_engine;

    bool have_last_scan_ts{false};
    uint64_t last_scan_ts_ns{0};

    ouster::sdk::core::ArrayX3fR lut_direction;
    ouster::sdk::core::ArrayX3fR lut_offset;
    ouster::sdk::core::PointCloudXYZf points_buffer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    pcl::PCLPointCloud2 staging_pcl_pc2;

    bool have_prev_pose{false};
    rclcpp::Time prev_stamp;
    Eigen::Vector3d prev_translation{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d prev_rotation{Eigen::Matrix3d::Identity()};

    std::shared_ptr<nav_msgs::msg::Odometry> odom_msg;
    std::shared_ptr<nav_msgs::msg::Path> trajectory_msg;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> map_msg;
};

}  // namespace ouster_ros
