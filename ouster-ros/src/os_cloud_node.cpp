/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_node.cpp
 * @brief A node to publish point clouds and imu topics
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ouster_sensor_msgs/msg/packet_msg.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

#include "os_static_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"
#include "point_cloud_processor_factory.h"
#include "telemetry_handler.h"

namespace ouster_ros {

using ouster_sensor_msgs::msg::PacketMsg;
using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::LidarPacket;

class OusterCloud : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterCloud(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_cloud", options), tf_bcast(*this) {
        on_init();
    }

    ~OusterCloud() override { stop_pipeline(); }

   private:
    bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void on_init() {
        declare_parameters();
        tf_bcast.parse_parameters();
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterCloud: node initialized!");
    }

    void declare_parameters() {
        tf_bcast.declare_parameters();
        declare_parameter("timestamp_mode", "");
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("proc_mask", "IMU|PCL|SCAN");
        declare_parameter("use_system_default_qos", false);
        declare_parameter("scan_ring", 0);
        declare_parameter("point_type", "original");
        declare_parameter("organized", true);
        declare_parameter("destagger", true);
        declare_parameter("min_range", 0.0);
        declare_parameter("max_range", 1000.0);
        declare_parameter("v_reduction", 1);
        declare_parameter("min_scan_valid_columns_ratio", 0.0);
        declare_parameter("mask_path", "");
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        if (metadata_msg->data == active_metadata_) {
            RCLCPP_DEBUG(get_logger(),
                         "OusterCloud: ignoring unchanged sensor metadata");
            return;
        }

        try {
            auto parsed_info =
                ouster::sdk::core::SensorInfo(metadata_msg->data);
            auto parsed_format =
                std::make_shared<ouster::sdk::core::PacketFormat>(
                    ouster::sdk::core::get_format(parsed_info));

            stop_pipeline();
            info = std::move(parsed_info);
            packet_format = std::move(parsed_format);
            if (tf_bcast.publish_static_tf()) {
                tf_bcast.broadcast_transforms(info);
            }
            create_publishers_subscriptions(info);
            active_metadata_ = metadata_msg->data;
        } catch (const std::exception& e) {
            stop_pipeline();
            active_metadata_.clear();
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "OusterCloud: failed to activate sensor metadata: "
                    << e.what());
        }
    }

    void create_publishers_subscriptions(const ouster::sdk::core::SensorInfo& info) {
        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();

        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS selected_qos =
            use_system_default_qos ?
                static_cast<rclcpp::QoS>(rclcpp::SystemDefaultsQoS()) :
                static_cast<rclcpp::QoS>(rclcpp::SensorDataQoS());

        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = impl::parse_tokens(proc_mask, '|');

        if (impl::check_token(tokens, "IMU")) {
            imu_pub =
                create_publisher<sensor_msgs::msg::Imu>("imu", selected_qos);
            imu_packet_handler = ImuPacketHandler::create(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
            imu_packet_buffer =
                std::make_unique<ImuPacket>(packet_format->imu_packet_size);
            imu_packet_buffer->format = packet_format;
            imu_packet_sub = create_subscription<PacketMsg>(
                "imu_packets",
                rclcpp::QoS(selected_qos).keep_last(info.format.imu_packets_per_frame),
                [this](const PacketMsg::ConstSharedPtr msg) {
                    if (imu_packet_handler) {
                        if (!packet_format ||
                            msg->buf.size() < packet_format->imu_packet_size) {
                            RCLCPP_WARN_STREAM_THROTTLE(
                                get_logger(), *get_clock(), 1000,
                                "dropping undersized imu_packets msg ("
                                    << msg->buf.size() << " bytes)");
                            return;
                        }
                        auto& imu_packet = *imu_packet_buffer;
                        imu_packet.host_timestamp =
                            static_cast<uint64_t>(now().nanoseconds());
                        memcpy(imu_packet.buf.data(), msg->buf.data(),
                               imu_packet.buf.size());
                        auto imu_msgs = imu_packet_handler(imu_packet);
                        for (const auto& imu_msg : imu_msgs) {
                            imu_pub->publish(imu_msg);
                        }
                    }
                });
        }

        auto min_scan_valid_columns_ratio = get_parameter("min_scan_valid_columns_ratio").as_double();
        if (min_scan_valid_columns_ratio < 0.0f || min_scan_valid_columns_ratio > 1.0f) {
            RCLCPP_FATAL(get_logger(), "min_scan_valid_columns_ratio needs to be in the range [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio out of bounds!");
        }

        int num_returns = info.num_returns();

        std::vector<LidarScanProcessor> processors;
        bool needs_rgb = false;

        if (impl::check_token(tokens, "PCL")) {
            rclcpp::PublisherOptions point_cloud_pub_options;
            point_cloud_pub_options.qos_overriding_options =
                rclcpp::QosOverridingOptions::with_default_policies();
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                    topic_for_return("points", i), selected_qos,
                    point_cloud_pub_options);
            }

            auto point_type = get_parameter("point_type").as_string();
            needs_rgb =
                PointCloudProcessorFactory::point_type_produces_color(point_type);
            auto organized = get_parameter("organized").as_bool();
            auto destagger = get_parameter("destagger").as_bool();
            auto min_range_m = get_parameter("min_range").as_double();
            auto max_range_m = get_parameter("max_range").as_double();
            if (min_range_m < 0.0 || max_range_m < 0.0) {
                RCLCPP_FATAL(get_logger(), "min_range and max_range need to be positive");
                throw std::runtime_error("negative range limits!");
            }
            if (min_range_m >= max_range_m) {
                RCLCPP_FATAL(get_logger(), "min_range can't be equal or exceed max_range");
                throw std::runtime_error("min_range equal to or exceeds max_range!");
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

            auto mask_path = get_parameter("mask_path").as_string();

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
            // TODO: avoid this duplication in os_cloud_node
            int beams_count = static_cast<int>(get_beams_count(info));
            int scan_ring = get_parameter("scan_ring").as_int();
            scan_ring = std::min(std::max(scan_ring, 0), beams_count - 1);
            if (scan_ring != get_parameter("scan_ring").as_int()) {
                RCLCPP_WARN_STREAM(get_logger(),
                    "scan ring is set to a value that exceeds available range"
                    "please choose a value between [0, " << beams_count << "], "
                    "ring value clamped to: " << scan_ring);
            }

            processors.push_back(LaserScanProcessor::create(
                info, tf_bcast.lidar_frame_id(), scan_ring,
                [this](LaserScanProcessor::OutputType msgs) {
                    for (size_t i = 0; i < msgs.size(); ++i) scan_pubs[i]->publish(*msgs[i]);
                }));
        }

        if (impl::check_token(tokens, "PCL") || impl::check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
                min_scan_valid_columns_ratio, needs_rgb);
        }

        if (impl::check_token(tokens, "TLM")) {
            telemetry_pub =
                create_publisher<ouster_sensor_msgs::msg::Telemetry>("telemetry",
                                                                     selected_qos);
            telemetry_handler = TelemetryHandler::create(
                info, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
        }

        if (impl::check_token(tokens, "PCL") ||
            impl::check_token(tokens, "SCAN") ||
            impl::check_token(tokens, "TLM")) {
            lidar_packet_buffer =
                std::make_unique<LidarPacket>(packet_format->lidar_packet_size);
            lidar_packet_buffer->format = packet_format;
            lidar_packet_sub = create_subscription<PacketMsg>(
                "lidar_packets",
                rclcpp::QoS(selected_qos).keep_last(lidar_packets_per_frame(info)),
                [this](const PacketMsg::ConstSharedPtr msg) {
                    if (!packet_format ||
                        msg->buf.size() < packet_format->lidar_packet_size) {
                        RCLCPP_WARN_STREAM_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            "dropping undersized lidar_packets msg ("
                                << msg->buf.size() << " bytes)");
                        return;
                    }
                    auto& lidar_packet = *lidar_packet_buffer;
                    lidar_packet.host_timestamp =
                        static_cast<uint64_t>(now().nanoseconds());
                    memcpy(lidar_packet.buf.data(), msg->buf.data(),
                           lidar_packet.buf.size());

                    if (telemetry_handler) {
                        auto telemetry = telemetry_handler(lidar_packet);
                        telemetry_pub->publish(telemetry);
                    }

                    if (lidar_packet_handler) {
                        lidar_packet_handler(lidar_packet);
                    }
                });
        }
    }

    void stop_pipeline() {
        lidar_packet_sub.reset();
        imu_packet_sub.reset();
        lidar_packet_handler = nullptr;
        imu_packet_handler = nullptr;
        telemetry_handler = nullptr;
        lidar_packet_buffer.reset();
        imu_packet_buffer.reset();
        lidar_pubs.clear();
        scan_pubs.clear();
        imu_pub.reset();
        telemetry_pub.reset();
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr imu_packet_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr>
        scan_pubs;

    OusterStaticTransformsBroadcaster<rclcpp::Node> tf_bcast;

    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
    std::unique_ptr<ImuPacket> imu_packet_buffer;
    std::unique_ptr<LidarPacket> lidar_packet_buffer;

    rclcpp::Publisher<ouster_sensor_msgs::msg::Telemetry>::SharedPtr telemetry_pub;
    TelemetryHandler::HandlerType telemetry_handler;
    std::string active_metadata_;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
