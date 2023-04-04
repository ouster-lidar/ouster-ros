/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_ros.h
 * @brief Higher-level functions to read data from the ouster sensors as ROS
 * messages
 */

#pragma once

#define PCL_NO_PRECOMPILE
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ouster/client.h>
#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <chrono>
#include <string>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_ros/os_point.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using Cloud = pcl::PointCloud<Point>;
using ns = std::chrono::nanoseconds;

/**
 * Read an imu packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param[in] cli the sensor client
 * @param[out] pm the destination packet message
 * @param[in] pf the packet format
 * @return whether reading was successful
 */
bool read_imu_packet(const sensor::client& cli, ouster_msgs::msg::PacketMsg& pm,
                     const sensor::packet_format& pf);

/**
 * Read a lidar packet into a ROS message. Blocks for up to a second if no data
 * is available.
 * @param[in] cli the sensor client
 * @param[out] pm the destination packet message
 * @param[in] pf the packet format
 * @return whether reading was successful
 */
bool read_lidar_packet(const sensor::client& cli,
                       ouster_msgs::msg::PacketMsg& pm,
                       const sensor::packet_format& pf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param[in] pm packet message populated by read_imu_packet
 * @param[in] timestamp the timestamp to give the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @param[in] pf the packet format
 * @return ROS sensor message with fields populated from the packet
 */
sensor_msgs::msg::Imu packet_to_imu_msg(const ouster_msgs::msg::PacketMsg& pm,
                                        const rclcpp::Time& timestamp,
                                        const std::string& frame,
                                        const sensor::packet_format& pf);

/**
 * Populate a PCL point cloud from a LidarScan.
 * @param[in, out] points The points parameters is used to store the results of
 * the cartesian product before it gets packed into the cloud object.
 * @param[in] lut_direction the direction of the xyz lut (with single precision)
 * @param[in] lut_offset the offset of the xyz lut (with single precision)
 * @param[in] scan_ts scan start used to caluclate relative timestamps for
 * points.
 * @param[in] lidar_scan input lidar data
 * @param[out] cloud output pcl pointcloud to populate
 * @param[in] return_index index of return desired starting at 0
 */
[[deprecated("use the 2nd version of scan_to_cloud_f")]] void scan_to_cloud_f(
    ouster::PointsF& points, const ouster::PointsF& lut_direction,
    const ouster::PointsF& lut_offset, std::chrono::nanoseconds scan_ts,
    const ouster::LidarScan& lidar_scan, ouster_ros::Cloud& cloud, int return_index);

/**
 * Populate a PCL point cloud from a LidarScan.
 * @param[in, out] points The points parameters is used to store the results of
 * the cartesian product before it gets packed into the cloud object.
 * @param[in] lut_direction the direction of the xyz lut (with single precision)
 * @param[in] lut_offset the offset of the xyz lut (with single precision)
 * @param[in] scan_ts scan start used to caluclate relative timestamps for
 * points
 * @param[in] lidar_scan input lidar data
 * @param[out] cloud output pcl pointcloud to populate
 * @param[in] return_index index of return desired starting at 0
 */
void scan_to_cloud_f(ouster::PointsF& points,
                     const ouster::PointsF& lut_direction,
                     const ouster::PointsF& lut_offset, uint64_t scan_ts,
                     const ouster::LidarScan& lidar_scan, ouster_ros::Cloud& cloud,
                     int return_index);

/**
 * Serialize a PCL point cloud to a ROS message
 * @param[in] cloud the PCL point cloud to convert
 * @param[in] timestamp the timestamp to apply to the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @return a ROS message containing the point cloud
 */
sensor_msgs::msg::PointCloud2 cloud_to_cloud_msg(const Cloud& cloud,
                                                 const rclcpp::Time& timestamp,
                                                 const std::string& frame);

/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param[in] mat transformation matrix return by sensor
 * @param[in] frame the parent frame of the published transform
 * @param[in] child_frame the child frame of the published transform
 * @param[in] timestamp value to set as the timestamp of the generated
 * TransformStamped message
 * @return ROS message suitable for publishing as a transform
 */
geometry_msgs::msg::TransformStamped transform_to_tf_msg(
    const ouster::mat4d& mat, const std::string& frame,
    const std::string& child_frame, rclcpp::Time timestamp);
}  // namespace ouster_ros
