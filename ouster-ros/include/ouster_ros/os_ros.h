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
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <chrono>
#include <string>

#include "ouster_sensor_msgs/msg/packet_msg.hpp"
#include "ouster_sensor_msgs/msg/telemetry.hpp"
#include "ouster_ros/os_point.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;

/**
 * Checks sensor_info if it currently represents a legacy udp lidar profile
 * @param[in] info sensor_info
 * @return whether sensor_info represents the legacy udp lidar profile
 */
bool is_legacy_lidar_profile(const sensor::sensor_info& info);

/**
 * Gets the number of point cloud returns that this sensor_info object represents
 * @param[in] info sensor_info
 * @return number of returns
 */
int get_n_returns(const sensor::sensor_info& info);


/**
 * Gets the number beams based on supplied sensor_info
 * @param[in] info sensor_info
 * @return number of beams a sensor has
 */
size_t get_beams_count(const sensor::sensor_info& info);

/**
 * Adds a suffix to the topic base name based on the return index
 * @param[in] topic_base topic base name
 * @param[in] return_idx return index {0, 1, ... n_returns }
 * @return number of returns
 */
std::string topic_for_return(const std::string& topic_base, int return_idx);

/**
 * Parse an imu packet message into a ROS imu message
 * @param[in] pf the packet format
 * @param[in] timestamp the timestamp to give the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @param[in] buf the raw packet message populated by read_imu_packet
 * @return ROS sensor message with fields populated from the packet
 */
sensor_msgs::msg::Imu packet_to_imu_msg(const ouster::sensor::packet_format& pf,
                                        const rclcpp::Time& timestamp,
                                        const std::string& frame,
                                        const uint8_t* buf);

/**
 * Parse an imu packet message into a ROS imu message
 * @param[in] pm packet message populated by read_imu_packet
 * @param[in] timestamp the timestamp to give the resulting ROS message
 * @param[in] frame the frame to set in the resulting ROS message
 * @param[in] pf the packet format
 * @return ROS sensor message with fields populated from the packet
 */
sensor_msgs::msg::Imu packet_to_imu_msg(const ouster_sensor_msgs::msg::PacketMsg& pm,
                                        const rclcpp::Time& timestamp,
                                        const std::string& frame,
                                        const sensor::packet_format& pf);

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


/**
 * Convert transformation matrix return by sensor to ROS transform
 * @param[in] ls lidar scan object
 * @param[in] timestamp value to set as the timestamp of the generated
 * @param[in] frame the parent frame of the generated laser scan message
 * @param[in] lidar_mode lidar mode (width x frequency)
 * @param[in] ring selected ring to be published
 * @param[in] pixel_shift_by_row pixel shifts by row
 * @param[in] return_index index of return desired starting at 0
 * @return ROS message suitable for publishing as a LaserScan
 */
sensor_msgs::msg::LaserScan lidar_scan_to_laser_scan_msg(
    const ouster::LidarScan& ls,
    const rclcpp::Time& timestamp,
    const std::string &frame,
    const ouster::sensor::lidar_mode lidar_mode,
    const uint16_t ring, const std::vector<int>& pixel_shift_by_row,
    const int return_index);

/**
 * Parse a LidarPacket and generate the Telemetry message
 * @param[in] lidar_packet lidar packet to parse telemetry data from
 * @param[in] timestamp the timestamp to give the resulting ROS message
 * @param[in] pf the packet format
 * @return ROS sensor message with fields populated from the packet
 */
ouster_sensor_msgs::msg::Telemetry lidar_packet_to_telemetry_msg(
    const ouster::sensor::LidarPacket& lidar_packet,
    const rclcpp::Time& timestamp,
    const ouster::sensor::packet_format& pf);

namespace impl {

sensor::ChanField scan_return(sensor::ChanField input_field, bool second);

struct read_and_cast {
    template <typename T, typename U>
    void operator()(Eigen::Ref<const ouster::img_t<T>> field,
                    ouster::img_t<U>& dest) {
        dest = field.template cast<U>();
    }
};

template <typename T>
inline ouster::img_t<T> get_or_fill_zero(sensor::ChanField field,
                                         const ouster::LidarScan& ls) {
    if (!ls.field_type(field)) return ouster::img_t<T>::Zero(ls.h, ls.w);
    ouster::img_t<T> result{ls.h, ls.w};
    ouster::impl::visit_field(ls, field, read_and_cast(), result);
    return result;
}

/**
 * simple utility function that ensures we don't wrap around uint64_t due
 * to a negative value being bigger than ts value in absolute terms.
 * @remark method does not check upper boundary
 */
inline uint64_t ts_safe_offset_add(uint64_t ts, int64_t offset) {
    return offset < 0 && ts < static_cast<uint64_t>(std::abs(offset)) ? 0 : ts + offset;
}

std::set<std::string> parse_tokens(const std::string& input, char delim);

inline bool check_token(const std::set<std::string>& tokens,
                        const std::string& token) {
    return tokens.find(token) != tokens.end();
}

ouster::util::version parse_version(const std::string& fw_rev);

template <typename T>
uint64_t ulround(T value) {
    T rounded_value = std::round(value);
    if (rounded_value < 0) return 0ULL;
    if (rounded_value > ULLONG_MAX) return ULLONG_MAX;
    return static_cast<uint64_t>(rounded_value);
}

void warn_mask_resized(int image_cols, int image_rows,
                       int scan_height, int scan_width);

template <typename pixel_type>
ouster::img_t<pixel_type> load_mask(const std::string& mask_path,
                                    size_t height, size_t width) {
    if (mask_path.empty()) return ouster::img_t<pixel_type>();

    cv::Mat image = cv::imread(mask_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        throw std::runtime_error("Failed to load mask image from path: " + mask_path);
    }

    if (image.rows != static_cast<int>(height) || image.cols != static_cast<int>(width)) {
        warn_mask_resized(image.cols, image.rows, static_cast<int>(height), static_cast<int>(width));
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
        image = resized;
    }
    Eigen::MatrixXi eigen_img(image.rows, image.cols);
    cv::cv2eigen(image, eigen_img);
    Eigen::MatrixXi zero_image = Eigen::MatrixXi::Zero(eigen_img.rows(), eigen_img.cols());
    Eigen::MatrixXi ones_image = Eigen::MatrixXi::Ones(eigen_img.rows(), eigen_img.cols());
    return (eigen_img.array() == 0.0).select(zero_image, ones_image).cast<pixel_type>();
}

} // namespace impl

}  // namespace ouster_ros
