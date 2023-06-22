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
#include "image_processor.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;

class OusterDriver : public OusterSensor {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterDriver(const rclcpp::NodeOptions& options)
        : OusterSensor("os_driver", options), os_tf_bcast(this) {
        os_tf_bcast.declare_parameters();
        os_tf_bcast.parse_parameters();
        declare_parameter<std::string>("proc_mask", "IMU|IMG|PCL|SCAN");
        declare_parameter<int>("scan_ring", 0);
    }

    LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "os_driver::on_activate() is called.");
        auto cb_return = OusterSensor::on_activate(state);
        if (cb_return != LifecycleNodeInterface::CallbackReturn::SUCCESS)
            return cb_return;
        imu_pub->on_activate();
        for (auto p : lidar_pubs) p->on_activate();
        for (auto p : scan_pubs) p->on_activate();
        for (auto p : image_pubs) p.second->on_activate();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    virtual void on_metadata_updated(const sensor::sensor_info& info) override {
        OusterSensor::on_metadata_updated(info);
        os_tf_bcast.broadcast_transforms(info);
    }

    virtual void create_publishers() override {
        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = parse_tokens(proc_mask, '|');

        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
        rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
        auto selected_qos =
            use_system_default_qos ? system_default_qos : sensor_data_qos;

        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        bool use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME" ||
                            timestamp_mode_arg == "TIME_FROM_ROS_RECEPTION";

        if (check_token(tokens, "IMU")) {
            imu_pub =
                create_publisher<sensor_msgs::msg::Imu>("imu", selected_qos);
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, os_tf_bcast.imu_frame_id(), use_ros_time);
        }

        int num_returns = get_n_returns(info);

        std::vector<LidarScanProcessor> processors;
        if (check_token(tokens, "PCL")) {
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_for_return("points", i), selected_qos);
            }

            processors.push_back(PointCloudProcessor::create(
                info, os_tf_bcast.point_cloud_frame_id(),
                os_tf_bcast.apply_lidar_to_sensor_transform(),
                [this](PointCloudProcessor::OutputType msgs) {
                    for (size_t i = 0; i < msgs.size(); ++i)
                        lidar_pubs[i]->publish(*msgs[i]);
                }));
        }

        if (check_token(tokens, "SCAN")) {
            scan_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                scan_pubs[i] = create_publisher<sensor_msgs::msg::LaserScan>(
                    topic_for_return("scan", i), selected_qos);
            }

            // TODO: avoid duplication in os_cloud_node
            int beams_count = static_cast<int>(get_beams_count(info));
            int scan_ring = get_parameter("scan_ring").as_int();
            scan_ring = std::min(std::max(scan_ring, 0), beams_count - 1);
            if (scan_ring != get_parameter("scan_ring").as_int()) {
                RCLCPP_WARN_STREAM(
                    get_logger(),
                    "scan ring is set to a value that exceeds available range"
                    "please choose a value between [0, "
                        << beams_count
                        << "], "
                           "ring value clamped to: "
                        << scan_ring);
            }

            processors.push_back(LaserScanProcessor::create(
                info,
                os_tf_bcast
                    .point_cloud_frame_id(),  // TODO: should we have different
                                              // frame for the laser scan than
                                              // point cloud???
                scan_ring, [this](LaserScanProcessor::OutputType msgs) {
                    for (size_t i = 0; i < msgs.size(); ++i)
                        scan_pubs[i]->publish(*msgs[i]);
                }));
        }

        if (check_token(tokens, "IMG")) {
            const std::map<sensor::ChanField, std::string>
                channel_field_topic_map_1{
                    {sensor::ChanField::RANGE, "range_image"},
                    {sensor::ChanField::SIGNAL, "signal_image"},
                    {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                    {sensor::ChanField::NEAR_IR, "nearir_image"}};

            const std::map<sensor::ChanField, std::string>
                channel_field_topic_map_2{
                    {sensor::ChanField::RANGE, "range_image"},
                    {sensor::ChanField::SIGNAL, "signal_image"},
                    {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                    {sensor::ChanField::NEAR_IR, "nearir_image"},
                    {sensor::ChanField::RANGE2, "range_image2"},
                    {sensor::ChanField::SIGNAL2, "signal_image2"},
                    {sensor::ChanField::REFLECTIVITY2, "reflec_image2"}};

            auto which_map = num_returns == 1 ? &channel_field_topic_map_1
                                              : &channel_field_topic_map_2;
            for (auto it = which_map->begin(); it != which_map->end(); ++it) {
                image_pubs[it->first] =
                    create_publisher<sensor_msgs::msg::Image>(it->second,
                                                              selected_qos);
            }

            processors.push_back(ImageProcessor::create(
                info, os_tf_bcast.point_cloud_frame_id(),
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                }));
        }

        if (check_token(tokens, "PCL") || check_token(tokens, "SCAN") ||
            check_token(tokens, "IMG"))
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

    virtual void cleanup() override {
        imu_packet_handler = nullptr;
        lidar_packet_handler = nullptr;
        imu_pub.reset();
        for (auto p : lidar_pubs) p.reset();
        for (auto p : scan_pubs) p.reset();
        for (auto p : image_pubs) p.second.reset();
        OusterSensor::cleanup();
    }

   private:
    OusterStaticTransformsBroadcaster<rclcpp_lifecycle::LifecycleNode>
        os_tf_bcast;

    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    std::vector<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    std::vector<rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        scan_pubs;
    std::map<sensor::ChanField,
             rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;
    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterDriver)
