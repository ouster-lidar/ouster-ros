/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_image_node.cpp
 * @brief A node to decode range, near ir and signal images from ouster
 * point cloud
 *
 * Publishes ~/range_image, ~/nearir_image, and ~/signal_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_processing_node_base.h"

#include "lidar_packet_handler.h"
#include "image_processor.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster_sensor_msgs::msg::PacketMsg;


class OusterImage : public OusterProcessingNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterImage(const rclcpp::NodeOptions& options)
        : OusterProcessingNodeBase("os_image", options) {
        on_init();
    }

   private:
    void on_init() {
        declare_parameter("timestamp_mode", "");
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("use_system_default_qos", false);
        declare_parameter("min_scan_valid_columns_ratio", 0.0);
        declare_parameter("mask_path", "");
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterImage: node initialized!");
    }

    void metadata_handler(const std_msgs::msg::String::ConstPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterImage: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        create_publishers_subscribers(get_n_returns(info));
    }

    void create_publishers_subscribers(int n_returns) {

        // TODO: avoid having to replicate the parameters: 
        // timestamp_mode, ptp_utc_tai_offset, use_system_default_qos in yet
        // another node.
        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();
        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
        rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
        auto selected_qos =
            use_system_default_qos ? system_default_qos : sensor_data_qos;

        const std::map<sensor::ChanField, std::string>
            channel_field_topic_map_1 {
                {sensor::ChanField::RANGE, "range_image"},
                {sensor::ChanField::SIGNAL, "signal_image"},
                {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                {sensor::ChanField::NEAR_IR, "nearir_image"}};

        const std::map<sensor::ChanField, std::string>
            channel_field_topic_map_2 {
                {sensor::ChanField::RANGE, "range_image"},
                {sensor::ChanField::SIGNAL, "signal_image"},
                {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                {sensor::ChanField::NEAR_IR, "nearir_image"},
                {sensor::ChanField::RANGE2, "range_image2"},
                {sensor::ChanField::SIGNAL2, "signal_image2"},
                {sensor::ChanField::REFLECTIVITY2, "reflec_image2"}};

        auto which_map = n_returns == 1 ? &channel_field_topic_map_1
                                        : &channel_field_topic_map_2;
        for (auto it = which_map->begin(); it != which_map->end(); ++it) {
            image_pubs[it->first] =
                create_publisher<sensor_msgs::msg::Image>(it->second,
                                                            selected_qos);
        }

        auto min_scan_valid_columns_ratio = get_parameter("min_scan_valid_columns_ratio").as_double();
        if (min_scan_valid_columns_ratio < 0.0f || min_scan_valid_columns_ratio > 1.0f) {
            RCLCPP_FATAL(get_logger(), "min_scan_valid_columns_ratio needs to be in the range [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio out of bounds!");
        }

        auto mask_path = get_parameter("mask_path").as_string();

        std::vector<LidarScanProcessor> processors {
            ImageProcessor::create(
                info, "os_lidar", /*TODO: tf_bcast.point_cloud_frame_id()*/
                mask_path,
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                })
        };

        lidar_packet_handler = LidarPacketHandler::create(
            info, processors, timestamp_mode,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            min_scan_valid_columns_ratio);
        lidar_packet_sub = create_subscription<PacketMsg>(
                "lidar_packets", selected_qos,
                [this](const PacketMsg::ConstSharedPtr msg) {
                    if (lidar_packet_handler) {
                        // TODO[UN]: this is not ideal since we can't reuse the msg buffer
                        // Need to redefine the Packet object and allow use of array_views
                        sensor::LidarPacket lidar_packet(msg->buf.size());
                        memcpy(lidar_packet.buf.data(), msg->buf.data(), msg->buf.size());
                        lidar_packet.host_timestamp = static_cast<uint64_t>(now().nanoseconds());
                        lidar_packet_handler(lidar_packet);
                    }
                });
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::map<sensor::ChanField,
             rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};
}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterImage)
