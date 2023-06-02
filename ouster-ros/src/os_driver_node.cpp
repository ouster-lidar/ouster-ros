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

#include <tf2_ros/static_transform_broadcaster.h>

#include "os_sensor_node.h"

#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"

namespace ouster_ros {

using ouster_msgs::msg::PacketMsg;

class OusterDriver : public OusterSensor {
    public:
    OUSTER_ROS_PUBLIC
    explicit OusterDriver(const rclcpp::NodeOptions& options)
        : OusterSensor("os_driver", options), tf_bcast(this) {
        declare_parameters();
        parse_parameters();
    }

private:
    void declare_parameters() {
        declare_parameter<std::string>("sensor_frame", "os_sensor");
        declare_parameter<std::string>("lidar_frame", "os_lidar");
        declare_parameter<std::string>("imu_frame", "os_imu");
        declare_parameter<std::string>("point_cloud_frame", "os_lidar");
    }

    void parse_parameters() {
        // TODO: avoid duplication of tf frames code
        sensor_frame = get_parameter("sensor_frame").as_string();
        lidar_frame = get_parameter("lidar_frame").as_string();
        imu_frame = get_parameter("imu_frame").as_string();
        point_cloud_frame = get_parameter("point_cloud_frame").as_string();

        // validate point_cloud_frame
        if (point_cloud_frame.empty()) {
            point_cloud_frame = lidar_frame;    // for ROS1 we'd still use sensor_frame
        } else if (point_cloud_frame != sensor_frame && point_cloud_frame != lidar_frame) {
            RCLCPP_WARN(get_logger(),
                     "point_cloud_frame value needs to match the value of either sensor_frame"
                     " or lidar_frame but a different value was supplied, using lidar_frame's"
                     " value as the value for point_cloud_frame");
            point_cloud_frame = lidar_frame;
        }
    }

    virtual void on_metadata_updated(const sensor::sensor_info& info) override {
        OusterSensor::on_metadata_updated(info);
        send_static_transforms(info);
    }

    void send_static_transforms(const sensor::sensor_info& info) {
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now()));
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now()));
    }

    virtual void create_publishers() override {
        int num_returns = get_n_returns(info);

        rclcpp::SensorDataQoS qos;
        imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", qos);
        lidar_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; i++) {
            lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_for_return("points", i), qos);
        }

        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        bool use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";

        imu_packet_handler = ImuPacketHandler::create_handler(
            info, imu_frame, use_ros_time);
        lidar_packet_handler = LidarPacketHandler::create_handler(
            info, point_cloud_frame, use_ros_time);
    }

    virtual void on_lidar_packet_msg(const uint8_t* raw_lidar_packet) override {
        auto point_cloud_msgs = lidar_packet_handler(raw_lidar_packet);
        for (size_t i = 0; i < point_cloud_msgs.size(); ++i) {
            lidar_pubs[i]->publish(*point_cloud_msgs[i]);
        }
    }

    virtual void on_imu_packet_msg(const uint8_t* raw_imu_packet) override {
        auto imu_msg = imu_packet_handler(raw_imu_packet);
        imu_pub->publish(imu_msg);
    }

private:
    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;
    std::string point_cloud_frame;

    tf2_ros::StaticTransformBroadcaster tf_bcast;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterDriver)
