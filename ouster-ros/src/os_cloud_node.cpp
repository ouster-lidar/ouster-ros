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
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <cassert>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_srvs/srv/get_metadata.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"

namespace sensor = ouster::sensor;
using ouster_msgs::msg::PacketMsg;

namespace ouster_ros {

class OusterCloud : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterCloud(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_cloud", options), tf_bcast(this) {
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
        declare_parameter<std::string>("sensor_frame", "os_sensor");
        declare_parameter<std::string>("lidar_frame", "os_lidar");
        declare_parameter<std::string>("imu_frame", "os_imu");
        declare_parameter<std::string>("timestamp_mode", "");
    }

    void parse_parameters() {
        sensor_frame = get_parameter("sensor_frame").as_string();
        lidar_frame = get_parameter("lidar_frame").as_string();
        imu_frame = get_parameter("imu_frame").as_string();

        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterCloud: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        send_static_transforms();
        create_publishers(get_n_returns());
        create_subscriptions();
    }

    void send_static_transforms() {
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now()));
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now()));
    }

    void create_publishers(int num_returns) {
        rclcpp::SensorDataQoS qos;
        imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", qos);
        lidar_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; i++) {
            lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_for_return("points", i), qos);
        }
    }

    void create_subscriptions() {
        rclcpp::SensorDataQoS qos;

        imu_packet_handler = ImuPacketHandler::create_handler(
            info, imu_frame, use_ros_time);
        imu_packet_sub = create_subscription<PacketMsg>(
            "imu_packets", qos,
            [this](const PacketMsg::ConstSharedPtr msg) {
                auto imu_msg = imu_packet_handler(msg->buf.data());
                imu_pub->publish(imu_msg);
            });

        lidar_packet_handler = LidarPacketHandler::create_handler(
            info, sensor_frame, use_ros_time);  // TODO: add an option to select sensor_frame
        lidar_packet_sub = create_subscription<PacketMsg>(
            "lidar_packets", qos,
            [this](const PacketMsg::ConstSharedPtr msg) {
                auto point_cloud_msgs = lidar_packet_handler(msg->buf.data());
                for (size_t i = 0; i < point_cloud_msgs.size(); ++i) {
                    lidar_pubs[i]->publish(*point_cloud_msgs[i]);
                }
            });
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;

    rclcpp::Subscription<PacketMsg>::SharedPtr imu_packet_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;

    tf2_ros::StaticTransformBroadcaster tf_bcast;

    bool use_ros_time;

    ImuPacketHandler::HandlerType imu_packet_handler;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
