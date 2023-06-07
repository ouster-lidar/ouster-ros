/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_driver.cpp
 * @brief This node combines the capabilities of os_sensor and os_cloud into
 * one ROS node/component
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include "os_sensor_node.h"

#include "os_static_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"

namespace ouster_ros {

using ouster_msgs::msg::PacketMsg;

class OusterDriver : public OusterSensor {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterDriver(const rclcpp::NodeOptions& options)
        : OusterSensor("os_driver", options), os_tf_bcast(this) {
        os_tf_bcast.declare_parameters();
        os_tf_bcast.parse_parameters();
        declare_parameter("proc_mask", std::string("IMU|PCL|SCAN"));
    }

    virtual void on_metadata_updated(const sensor::sensor_info& info) override {
        OusterSensor::on_metadata_updated(info);
        os_tf_bcast.broadcast_transforms(info);
    }

    virtual void create_publishers() override {
        int num_returns = get_n_returns(info);
        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = parse_tokens(proc_mask, '|');

        rclcpp::SensorDataQoS qos;

        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        bool use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME" ||
                            timestamp_mode_arg == "TIME_FROM_ROS_RECEPTION";

        if (check_token(tokens, "IMU")) {
            imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", qos);
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, os_tf_bcast.imu_frame_id(), use_ros_time);
        }

        std::vector<LidarScanProcessor> processors;
        if (check_token(tokens, "PCL")) {
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_for_return("points", i), qos);
            }

            processors.push_back(PointCloudProcessor::create(
                info, os_tf_bcast.point_cloud_frame_id(),
                os_tf_bcast.apply_lidar_to_sensor_transform(),
                [this](PointCloudProcessor::OutputType point_cloud_msg) {
                    for (size_t i = 0; i < point_cloud_msg.size(); ++i)
                        lidar_pubs[i]->publish(*point_cloud_msg[i]);
                }));
        }

        if (check_token(tokens, "SCAN")) {
            scan_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                scan_pubs[i] = create_publisher<sensor_msgs::msg::LaserScan>(
                    topic_for_return("scan", i), qos);
            }

            processors.push_back(LaserScanProcessor::create(
                info,
                os_tf_bcast
                    .point_cloud_frame_id(),  // TODO: should we have different
                                              // frame for the laser scan than
                                              // point cloud???
                0, [this](LaserScanProcessor::OutputType laser_scan_msg) {
                    for (size_t i = 0; i < laser_scan_msg.size(); ++i)
                        scan_pubs[i]->publish(*laser_scan_msg[i]);
                }));
        }

        if (check_token(tokens, "PCL") || check_token(tokens, "SCAN"))
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, use_ros_time, processors);
    }

    virtual void on_lidar_packet_msg(const uint8_t* raw_lidar_packet) override {
        if (lidar_packet_handler) lidar_packet_handler(raw_lidar_packet);
    }

    virtual void on_imu_packet_msg(const uint8_t* raw_imu_packet) override {
        if (imu_packet_handler)
            imu_pub->publish(imu_packet_handler(raw_imu_packet));
    }

   private:
    OusterStaticTransformsBroadcaster<rclcpp_lifecycle::LifecycleNode>
        os_tf_bcast;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        scan_pubs;
    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterDriver)
