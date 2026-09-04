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

#include <cstdint>
#include <cstring>
#include <exception>
#include <map>
#include <memory>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stdexcept>
#include <utility>

#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "lidar_packet_handler.h"
#include "image_processor.h"
#include "panorama_camera_info.h"
#include "static_transform_broadcaster_compat.h"

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

    ~OusterImage() override {
        lidar_packet_sub.reset();
        lidar_packet_handler = nullptr;
    }

   private:
    void on_init() {
        declare_parameter("timestamp_mode", "");
        declare_parameter("ptp_utc_tai_offset", -37.0);
        declare_parameter("use_system_default_qos", false);
        declare_parameter("min_scan_valid_columns_ratio", 0.0);
        declare_parameter("mask_path", "");
        declare_parameter("frame_id", "os_lidar");
        declare_parameter("optical_frame", "");
        // CameraInfo cannot fully describe the equirectangular projection, so
        // keep the approximation opt-in.
        declare_parameter("publish_camera_info", false);
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterImage: node initialized!");
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        std::lock_guard<std::mutex> pipeline_lock(pipeline_mutex);
        if (metadata_is_active(metadata_msg->data)) {
            RCLCPP_DEBUG(get_logger(),
                         "OusterImage: ignoring unchanged sensor metadata");
            return;
        }

        RCLCPP_INFO(get_logger(), "OusterImage: activating sensor metadata");
        try {
            auto parsed_info =
                ouster::sdk::core::SensorInfo(metadata_msg->data);
            auto parsed_format =
                std::make_shared<ouster::sdk::core::PacketFormat>(
                    ouster::sdk::core::get_format(parsed_info));
            info = parsed_info;
            packet_format = std::move(parsed_format);
            create_publishers_subscribers(info.num_returns());
            mark_metadata_active(metadata_msg->data);
        } catch (const std::exception& e) {
            // A different metadata message means the old decoder can no
            // longer be assumed to match incoming packets. Stop it before
            // reporting the failure so stale calibration cannot produce
            // plausible-looking images while waiting for valid metadata.
            begin_pipeline_update();
            lidar_packet_sub.reset();
            lidar_packet_handler = nullptr;
            lidar_packet_buffer.reset();
            image_pubs.clear();
            camera_info_pub_.reset();
            packet_format.reset();
            invalidate_active_metadata();
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "OusterImage: failed to activate sensor metadata: "
                    << e.what());
        }
    }

    void create_publishers_subscribers(int n_returns) {
        const uint64_t pipeline_generation = begin_pipeline_update();
        lidar_packet_sub.reset();
        lidar_packet_handler = nullptr;
        lidar_packet_buffer.reset();

        // TODO: avoid having to replicate the parameters:
        // timestamp_mode, ptp_utc_tai_offset, use_system_default_qos in yet
        // another node.
        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();
        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS selected_qos =
            use_system_default_qos ?
                static_cast<rclcpp::QoS>(rclcpp::SystemDefaultsQoS()) :
                static_cast<rclcpp::QoS>(rclcpp::SensorDataQoS());

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
        image_pubs.clear();
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
        auto frame_id = get_parameter("frame_id").as_string();
        auto optical_frame = get_parameter("optical_frame").as_string();
        publish_camera_info_ = get_parameter("publish_camera_info").as_bool();

        if (frame_id.empty()) {
            throw std::runtime_error("frame_id must not be empty");
        }
        if (publish_camera_info_ && optical_frame.empty()) {
            optical_frame = frame_id + "_panorama_optical_frame";
            RCLCPP_INFO(get_logger(),
                        "publish_camera_info is enabled; using derived "
                        "optical_frame '%s'",
                        optical_frame.c_str());
        }
        if (!optical_frame.empty() && optical_frame == frame_id) {
            throw std::runtime_error(
                "optical_frame must differ from frame_id");
        }

        const auto& image_frame = optical_frame.empty() ? frame_id
                                                        : optical_frame;
        if (!optical_frame.empty()) {
            broadcast_optical_transform(frame_id, optical_frame);
        }

        camera_info_pub_.reset();
        if (publish_camera_info_) {
            create_camera_info_publisher(info, image_frame, selected_qos);
        }

        std::vector<LidarScanProcessor> processors {
            ImageProcessor::create(
                info, image_frame,
                mask_path,
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first]->publish(*it->second);
                    }
                    if (publish_camera_info_) {
                        publish_camera_info(msgs);
                    }
                })
        };

        lidar_packet_handler = LidarPacketHandler::create(
            info, processors, timestamp_mode,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            min_scan_valid_columns_ratio);
        lidar_packet_buffer =
            std::make_unique<LidarPacket>(packet_format->lidar_packet_size);
        lidar_packet_buffer->format = packet_format;
        lidar_packet_sub = create_subscription<PacketMsg>(
                "lidar_packets",
                rclcpp::QoS(selected_qos).keep_last(lidar_packets_per_frame(info)),
                [this, pipeline_generation](
                    const PacketMsg::ConstSharedPtr& msg) {
                    std::lock_guard<std::mutex> pipeline_lock(pipeline_mutex);
                    if (!pipeline_is_current(pipeline_generation) ||
                        !lidar_packet_handler || !lidar_packet_buffer) {
                        return;
                    }
                    const size_t expected_size = packet_format->lidar_packet_size;
                    if (msg->buf.size() < expected_size) {
                        RCLCPP_WARN_STREAM_THROTTLE(
                            get_logger(), *get_clock(), 1000,
                            "dropping undersized lidar_packets msg ("
                                << msg->buf.size() << " < " << expected_size
                                << " bytes)");
                        return;
                    }
                    auto& lidar_packet = *lidar_packet_buffer;
                    lidar_packet.host_timestamp =
                        static_cast<uint64_t>(now().nanoseconds());
                    std::memcpy(lidar_packet.buf.data(), msg->buf.data(),
                                expected_size);
                    lidar_packet_handler(lidar_packet);
                });
    }

    void broadcast_optical_transform(const std::string& frame_id,
        const std::string& optical_frame) {
        // The destaggered center column faces lidar -X. Follow the optical
        // convention: Z forward, X right, Y down.
        tf2::Quaternion q;
        q.setRPY(-M_PI_2, 0.0, M_PI_2);

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = get_clock()->now();
        tf_msg.header.frame_id = frame_id;
        tf_msg.child_frame_id = optical_frame;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        if (!tf_bcast_) {
            tf_bcast_ = make_static_transform_broadcaster(this);
        }
        tf_bcast_->sendTransform(tf_msg);
    }

    void create_camera_info_publisher(
        const ouster::sdk::core::SensorInfo& sensor_info,
        const std::string& frame_id,
        const rclcpp::QoS& qos) {
        RCLCPP_WARN(get_logger(),
                    "os_image CameraInfo is an equirectangular approximation, "
                    "not a pinhole calibration; use os_pinhole for pinhole "
                    "projection or back-projection");
        auto result = make_panorama_camera_info(sensor_info, frame_id);
        if (result.used_vertical_fallback) {
            RCLCPP_WARN(get_logger(),
                        "CameraInfo needs at least two distinct beam altitude "
                        "angles; using a full-height fallback");
        }
        if (result.has_partial_column_window) {
            RCLCPP_WARN(
                get_logger(),
                "sensor uses a partial column_window; os_image retains the "
                "full destaggered raster with zero-filled unsupported pixels, "
                "so CameraInfo ROI remains full resolution");
        }
        camera_info_msg_ = std::move(result.camera_info);

        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", qos);
    }

    void publish_camera_info(const ImageProcessor::OutputType& msgs) {
        auto it = msgs.begin();
        if (it != msgs.end() && it->second) {
            camera_info_msg_.header.stamp = it->second->header.stamp;
        }
        camera_info_pub_->publish(camera_info_msg_);
    }

   private:
    std::map<std::string,
             rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        image_pubs;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;
    bool publish_camera_info_{false};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bcast_;

    std::unique_ptr<LidarPacket> lidar_packet_buffer;
    LidarPacketHandler::HandlerType lidar_packet_handler;
    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
};
}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterImage)
