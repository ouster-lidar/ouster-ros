/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_driver.cpp
 * @brief This node combines the capabilities of os_sensor, os_cloud and
 * os_image into a single ROS node/component
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
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
#include "point_cloud_processor_factory.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;

class OusterDriver : public OusterSensor {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterDriver(const rclcpp::NodeOptions& options)
        : OusterSensor("os_driver", options), tf_bcast(this) {
        tf_bcast.declare_parameters();
        tf_bcast.parse_parameters();
        declare_parameter("proc_mask", "IMU|IMG|PCL|SCAN");
        declare_parameter("scan_ring", 0);
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("point_type", "original");
    }

    ~OusterDriver() override {
        RCLCPP_DEBUG(get_logger(), "OusterDriver::~OusterDriver() called");
        halt();
    }

    virtual void on_metadata_updated(const sensor::sensor_info& info) override {
        OusterSensor::on_metadata_updated(info);
        tf_bcast.broadcast_transforms(info);
    }

    virtual void create_publishers() override {
        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = impl::parse_tokens(proc_mask, '|');

        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
        rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
        auto selected_qos =
            use_system_default_qos ? system_default_qos : sensor_data_qos;

        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();

        if (impl::check_token(tokens, "IMU")) {
            imu_pub =
                create_publisher<sensor_msgs::msg::Imu>("imu", selected_qos);
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
        }

        int num_returns = get_n_returns(info);

        std::vector<LidarScanProcessor> processors;
        if (impl::check_token(tokens, "PCL")) {
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_for_return("points", i), selected_qos);
            }

            auto point_type = get_parameter("point_type").as_string();
            processors.push_back(
                PointCloudProcessorFactory::create_point_cloud_processor(point_type, info,
                    tf_bcast.point_cloud_frame_id(), tf_bcast.apply_lidar_to_sensor_transform(),
                    [this](PointCloudProcessor_OutputType msgs) {
                        for (size_t i = 0; i < msgs.size(); ++i) lidar_pubs[i]->publish(*msgs[i]);
                    }
                )
            );

            // warn about profile incompatibility
            if (PointCloudProcessorFactory::point_type_requires_intensity(point_type) &&
                !PointCloudProcessorFactory::profile_has_intensity(info.format.udp_profile_lidar)) {
                RCLCPP_WARN_STREAM(
                    get_logger(),
                    "selected point type '" << point_type
                    << "' is not compatible with the udp profile: "
                    << to_string(info.format.udp_profile_lidar));
            }
        }

        if (impl::check_token(tokens, "SCAN")) {
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
                info, tf_bcast.lidar_frame_id(), scan_ring,
                [this](LaserScanProcessor::OutputType msgs) {
                    for (size_t i = 0; i < msgs.size(); ++i) scan_pubs[i]->publish(*msgs[i]);
                }));
        }

        if (impl::check_token(tokens, "IMG")) {
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
                info, tf_bcast.point_cloud_frame_id(),
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                }));
        }

        if (impl::check_token(tokens, "PCL") || impl::check_token(tokens, "SCAN") ||
            impl::check_token(tokens, "IMG"))
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
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
    OusterStaticTransformsBroadcaster<rclcpp_lifecycle::LifecycleNode> tf_bcast;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        scan_pubs;
    std::map<sensor::ChanField,
             rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;
    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterDriver)
