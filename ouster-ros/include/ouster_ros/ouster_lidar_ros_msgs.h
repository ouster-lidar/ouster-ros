/**
 * @file ouster_lidar_ros_msgs.h
 * @brief Fill ouster_sensor_msgs LidarInfo / LidarScan from Ouster SDK type.
 */

#pragma once

#include <ouster/lidar_scan.h>
#include <ouster/types.h>
#include <lidar_msgs/msg/lidar_info.hpp>
#include <lidar_msgs/msg/lidar_scan.hpp>
#include <rclcpp/time.hpp>

#include <string>

namespace ouster_ros {

/**
 * Populate LidarInfo from SensorInfo (angles in radians, range raw in mm).
 */
void sensor_info_to_lidar_info(const ouster::sdk::core::SensorInfo& info,
                               const std::string& frame_id,
                               lidar_msgs::msg::LidarInfo& out);

/**
 * Pack SDK LidarScan pixel fields into non-interleaved lidar_msgs/LidarScan.
 * Channel names are lowercased SDK field tags (e.g. RANGE -> "range") and sorted
 * alphabetically to produce a stable wire layout across invocations.
 */
void ouster_sdk_lidar_scan_to_ros_msg(const ouster::sdk::core::LidarScan& ls,
                                      const rclcpp::Time& stamp,
                                      const std::string& frame_id,
                                      lidar_msgs::msg::LidarScan& out);

}  // namespace ouster_ros
