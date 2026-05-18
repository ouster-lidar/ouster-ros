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

#include <algorithm>
#include <sensor_msgs/msg/camera_info.hpp>

#include "lidar_packet_handler.h"
#include "image_processor.h"

namespace ouster_ros {

using ouster_sensor_msgs::msg::PacketMsg;
namespace ChanField = ouster::sdk::core::ChanField;
using ouster::sdk::core::LidarPacket;


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
        declare_parameter("distortion_model", "plumb_bob");
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterImage: node initialized!");
    }

    void metadata_handler(const std_msgs::msg::String::ConstPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterImage: retrieved new sensor metadata!");
        info = ouster::sdk::core::SensorInfo(metadata_msg->data);
        packet_format = std::make_shared<ouster::sdk::core::PacketFormat>(
            ouster::sdk::core::get_format(info));
        create_publishers_subscribers(info.num_returns());
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

        const std::map<std::string, std::string>
            channel_field_topic_map_1 {
                {ChanField::RANGE, "range_image"},
                {ChanField::SIGNAL, "signal_image"},
                {ChanField::REFLECTIVITY, "reflec_image"},
                {ChanField::NEAR_IR, "nearir_image"},
                {ChanField::RGB, "rgb_image"}};

        const std::map<std::string, std::string>
            channel_field_topic_map_2 {
                {ChanField::RANGE, "range_image"},
                {ChanField::SIGNAL, "signal_image"},
                {ChanField::REFLECTIVITY, "reflec_image"},
                {ChanField::NEAR_IR, "nearir_image"},
                {ChanField::RANGE2, "range_image2"},
                {ChanField::SIGNAL2, "signal_image2"},
                {ChanField::REFLECTIVITY2, "reflec_image2"},
                {ChanField::RGB, "rgb_image"}};

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

        create_camera_info_publisher(info, selected_qos);

        std::vector<LidarScanProcessor> processors {
            ImageProcessor::create(
                info, "os_lidar", /*TODO: tf_bcast.point_cloud_frame_id()*/
                mask_path,
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                    publish_camera_info(msgs);
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
                        LidarPacket lidar_packet(msg->buf.size());
                        lidar_packet.format = packet_format;
                        lidar_packet.host_timestamp = static_cast<uint64_t>(now().nanoseconds());
                        memcpy(lidar_packet.buf.data(), msg->buf.data(), msg->buf.size());
                        lidar_packet_handler(lidar_packet);
                    }
                });
    }

    void create_camera_info_publisher(
        const ouster::sdk::core::SensorInfo& sensor_info,
        const rclcpp::QoS& qos) {
        uint32_t H = sensor_info.format.pixels_per_column;
        uint32_t W = sensor_info.format.columns_per_frame;

        // Equirectangular projection for the LiDAR range image.
        // Horizontal: each column = 2π/W radians (full rotation, azimuth-window
        // only affects data validity, not the column ↔ angle mapping).
        double fx = static_cast<double>(W) / (2.0 * M_PI);
        double cx = static_cast<double>(W) / 2.0;

        // Vertical: use actual beam altitude angles for correct VFOV.
        // The beams are approximately uniformly spaced; per-beam non-uniformity
        // cannot be represented in a single fy but this is far closer than 2π.
        double fy;
        double cy;
        const auto& alts = sensor_info.beam_altitude_angles;
        if (alts.size() >= 2) {
            auto [min_it, max_it] = std::minmax_element(alts.begin(), alts.end());
            double vfov_rad = (*max_it - *min_it) * M_PI / 180.0;
            fy = static_cast<double>(H) / vfov_rad;
            // cy: row where elevation = 0°.  Beams are ordered top-to-bottom
            // (highest altitude = row 0), so 0° maps to:
            double mean_alt_rad = 0.5 * (*max_it + *min_it) * M_PI / 180.0;
            cy = static_cast<double>(H) / 2.0 - mean_alt_rad * fy;
        } else {
            fy = static_cast<double>(H) / (2.0 * M_PI);
            cy = static_cast<double>(H) / 2.0;
        }

        camera_info_msg_.header.frame_id = "os_lidar";
        camera_info_msg_.height = H;
        camera_info_msg_.width = W;
        camera_info_msg_.distortion_model = get_parameter("distortion_model").as_string();
        camera_info_msg_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0,
                              1.0, 0.0};

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos);
    }

    void publish_camera_info(const ImageProcessor::OutputType& msgs) {
        // Use the timestamp from any image message for synchronization
        auto it = msgs.begin();
        if (it != msgs.end() && it->second) {
            camera_info_msg_.header.stamp = it->second->header.stamp;
        }
        camera_info_pub_->publish(camera_info_msg_);
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::map<std::string,
             rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};
}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterImage)
