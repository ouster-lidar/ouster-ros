/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_node.cpp
 * @brief A node to publish point clouds and imu topics
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <cassert>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_srvs/srv/get_metadata.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

#include "os_static_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"

namespace sensor = ouster::sensor;
using ouster_msgs::msg::PacketMsg;

namespace ouster_ros {

class OusterCloud : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterCloud(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_cloud", options), os_tf_bcast(this) {
        on_init();
    }

   private:
    bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void on_init() {
        declare_parameters();
        parse_parameters();
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterCloud: node initialized!");
    }

    void declare_parameters() {
        os_tf_bcast.declare_parameters();
        declare_parameter<std::string>("timestamp_mode", "");
        declare_parameter("proc_mask", std::string("IMU|PCL|SCAN"));
    }

    void parse_parameters() {
        os_tf_bcast.parse_parameters();
        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME" ||
                       timestamp_mode_arg == "TIME_FROM_ROS_RECEPTION";
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterCloud: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        os_tf_bcast.broadcast_transforms(info);
        create_publishers(get_n_returns(info));
        create_subscriptions();
    }

    void create_publishers(int num_returns) {
        rclcpp::SensorDataQoS qos;
        imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", qos);
        lidar_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; i++) {
            lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_for_return("points", i), qos);
        }

        scan_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; i++) {
            scan_pubs[i] = create_publisher<sensor_msgs::msg::LaserScan>(
                topic_for_return("scan", i), qos);
        }
    }

    void create_subscriptions() {
        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = parse_tokens(proc_mask, '|');

        rclcpp::SensorDataQoS qos;

        if (check_token(tokens, "IMU")) {
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, os_tf_bcast.imu_frame_id(), use_ros_time);
            imu_packet_sub = create_subscription<PacketMsg>(
                "imu_packets", qos,
                [this](const PacketMsg::ConstSharedPtr msg) {
                    auto imu_msg = imu_packet_handler(msg->buf.data());
                    imu_pub->publish(imu_msg);
                });
        }

        std::vector<LidarScanProcessor> processors;
        if (check_token(tokens, "PCL")) {
            processors.push_back(PointCloudProcessor::create(
                info, os_tf_bcast.point_cloud_frame_id(),
                os_tf_bcast.apply_lidar_to_sensor_transform(),
                [this](PointCloudProcessor::OutputType point_cloud_msg) {
                    for (size_t i = 0; i < point_cloud_msg.size(); ++i)
                        lidar_pubs[i]->publish(*point_cloud_msg[i]);
                }));
        }

        if (check_token(tokens, "SCAN")) {
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

        if (check_token(tokens, "PCL") || check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, use_ros_time, processors);
            lidar_packet_sub = create_subscription<PacketMsg>(
                "lidar_packets", qos,
                [this](const PacketMsg::ConstSharedPtr msg) {
                    lidar_packet_handler(msg->buf.data());
                });
        }
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr imu_packet_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        scan_pubs;

    OusterStaticTransformsBroadcaster<rclcpp::Node> os_tf_bcast;
    bool use_ros_time;

    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
