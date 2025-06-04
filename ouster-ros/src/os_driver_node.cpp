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
#include "telemetry_handler.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster::sensor::LidarPacket;
using ouster::sensor::ImuPacket;
using ouster::sensor::LidarPacket;

class OusterDriver : public OusterSensor {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterDriver(const rclcpp::NodeOptions& options)
        : OusterSensor("os_driver", options), tf_bcast(this) {
        tf_bcast.declare_parameters();
        tf_bcast.parse_parameters();
        declare_parameter("proc_mask", "IMU|PCL|SCAN|IMG|RAW|TLM");
        declare_parameter("scan_ring", 0);
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("point_type", "original");
        declare_parameter("organized", true);
        declare_parameter("destagger", true);
        declare_parameter("min_range", 0.0);
        declare_parameter("max_range", 1000.0);
        declare_parameter("v_reduction", 1);
        declare_parameter("min_scan_valid_columns_ratio", 0.0);
        declare_parameter("mask_path", "");
    }

    ~OusterDriver() override {
        RCLCPP_DEBUG(get_logger(), "OusterDriver::~OusterDriver() called");
    }

    virtual void on_metadata_updated(const sensor::sensor_info& info) override {
        OusterSensor::on_metadata_updated(info);
        if (tf_bcast.publish_static_tf()) {
          tf_bcast.broadcast_transforms(info);
        }
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
            imu_packet_handler = ImuPacketHandler::create(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
        }

        auto min_scan_valid_columns_ratio = get_parameter("min_scan_valid_columns_ratio").as_double();
        if (min_scan_valid_columns_ratio < 0.0f || min_scan_valid_columns_ratio > 1.0f) {
            RCLCPP_FATAL(get_logger(), "min_scan_valid_columns_ratio needs to be in the range [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio out of bounds!");
        }

        auto mask_path = get_parameter("mask_path").as_string();

        int num_returns = get_n_returns(info);

        std::vector<LidarScanProcessor> processors;
        if (impl::check_token(tokens, "PCL")) {
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_for_return("points", i), selected_qos);
            }

            auto point_type = get_parameter("point_type").as_string();
            auto organized = get_parameter("organized").as_bool();
            auto destagger = get_parameter("destagger").as_bool();
            auto min_range_m = get_parameter("min_range").as_double();
            auto max_range_m = get_parameter("max_range").as_double();
            if (min_range_m < 0.0 || max_range_m < 0.0) {
                RCLCPP_FATAL(get_logger(), "min_range and max_range need to be positive");
                throw std::runtime_error("negative range limits!");
            }
            if (min_range_m >= max_range_m) {
                const auto error_msg = "min_range can't be equal or exceed max_range";
                RCLCPP_FATAL(get_logger(), error_msg);
                throw std::runtime_error(error_msg);
            }
            // convert to millimeters
            uint32_t min_range = impl::ulround(min_range_m * 1000);
            uint32_t max_range = impl::ulround(max_range_m * 1000);
            auto v_reduction = get_parameter("v_reduction").as_int();
            auto valid_values = std::vector<int>{1, 2, 4, 8, 16};
            if (std::find(valid_values.begin(), valid_values.end(),
                          v_reduction) == valid_values.end()) {
                RCLCPP_FATAL(get_logger(),
                    "v_reduction needs to be one of the values: {1, 2, 4, 8, 16}");
                throw std::runtime_error("invalid v_reduction value!");
            }

            processors.push_back(
                PointCloudProcessorFactory::create_point_cloud_processor(point_type,
                    info, tf_bcast.point_cloud_frame_id(),
                    tf_bcast.apply_lidar_to_sensor_transform(),
                    organized, destagger, min_range, max_range, v_reduction, mask_path,
                    [this](PointCloudProcessor_OutputType msgs) {
                        for (size_t i = 0; i < msgs.size(); ++i)
                            lidar_pubs[i]->publish(*msgs[i]);
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
                info, tf_bcast.point_cloud_frame_id(), mask_path,
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                }));
        }

        if (impl::check_token(tokens, "PCL") || impl::check_token(tokens, "SCAN") ||
            impl::check_token(tokens, "IMG"))
            lidar_packet_handler = LidarPacketHandler::create(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
                min_scan_valid_columns_ratio);

        if (impl::check_token(tokens, "TLM")) {
            telemetry_pub =
                create_publisher<ouster_sensor_msgs::msg::Telemetry>("telemetry",
                                                                     selected_qos);
            telemetry_handler = TelemetryHandler::create(
                info, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
        }

        publish_raw = impl::check_token(tokens, "RAW");
        if (publish_raw)
            OusterSensor::create_publishers();
    }

    virtual void on_lidar_packet_msg(const LidarPacket& lidar_packet) override {
        if (telemetry_handler) {
            auto telemetry = telemetry_handler(lidar_packet);
            telemetry_pub->publish(telemetry);
        }

        if (lidar_packet_handler)
            lidar_packet_handler(lidar_packet);

        if (publish_raw)
            OusterSensor::on_lidar_packet_msg(lidar_packet);
    }

    virtual void on_imu_packet_msg(const ImuPacket& imu_packet) override {
        if (imu_packet_handler)
            imu_pub->publish(imu_packet_handler(imu_packet));

        if (publish_raw)
            OusterSensor::on_imu_packet_msg(imu_packet);
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

    bool publish_raw = false;

    rclcpp::Publisher<ouster_sensor_msgs::msg::Telemetry>::SharedPtr telemetry_pub;
    TelemetryHandler::HandlerType telemetry_handler;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterDriver)
