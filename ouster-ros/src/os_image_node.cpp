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

#if __has_include(<tf2/LinearMath/Quaternion.hpp>)
#include <tf2/LinearMath/Quaternion.hpp>
#else
#include <tf2/LinearMath/Quaternion.h>
#endif
#include <tf2_ros/static_transform_broadcaster.h>

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
        declare_parameter("frame_id", "os_lidar");
        declare_parameter("optical_frame", "");
        declare_parameter("publish_camera_info", true);
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
        auto frame_id = get_parameter("frame_id").as_string();
        auto optical_frame = get_parameter("optical_frame").as_string();
        publish_camera_info_ = get_parameter("publish_camera_info").as_bool();

        // When optical_frame is set, stamp images and camera_info in that
        // frame and broadcast a static transform frame_id -> optical_frame.
        // When empty (default) behavior is unchanged: messages are stamped
        // with frame_id and no transform is published.
        const auto& image_frame = optical_frame.empty() ? frame_id
                                                        : optical_frame;
        if (!optical_frame.empty()) {
            broadcast_optical_transform(info, frame_id, optical_frame);
        }

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

    void broadcast_optical_transform(
        const ouster::sdk::core::SensorInfo& sensor_info,
        const std::string& frame_id,
        const std::string& optical_frame) {
        double W = static_cast<double>(sensor_info.format.columns_per_frame);

        // Azimuth of the image center column (cx = W/2) in the lidar frame.
        // Per the SDK xyz lut (ouster_client/src/xyzlut.cpp), raw column m has
        // encoder angle theta_e = 2*pi - m*(2*pi/W) plus a per-beam azimuth
        // theta_a = -beam_azimuth_angles[u]; destaggering shifts each row by
        // pixel_shift_by_row[u] ~= beam_azimuth*W/360 which cancels theta_a,
        // so destaggered image column v looks along azimuth
        // theta(v) = 2*pi - v*(2*pi/W) for every row. For v = cx = W/2 this
        // gives theta = pi, i.e. the image center looks along -X_lidar.
        double az_center = 2.0 * M_PI - (W / 2.0) * (2.0 * M_PI / W);

        // Rotate lidar axes to optical axes for a camera looking along
        // +X_lidar (roll = -pi/2, yaw = -pi/2), then yaw the optical axis
        // about Z_lidar to the center-column azimuth. With setRPY(r, p, y)
        // composing as Rz(y)*Ry(p)*Rx(r) this collapses to:
        tf2::Quaternion q;
        q.setRPY(-M_PI_2, 0.0, az_center - M_PI_2);
        // Resulting matrix (columns = optical axes in lidar coordinates):
        //   [ 0  0 -1 ]
        //   [ 1  0  0 ]   => Z_opt = -X_lidar (center column, forward),
        //   [ 0 -1  0 ]      Y_opt = -Z_lidar (down), X_opt = +Y_lidar.

        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = get_clock()->now();
        tf_msg.header.frame_id = frame_id;
        tf_msg.child_frame_id = optical_frame;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        if (!tf_bcast_) {
            tf_bcast_ =
                std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        }
        tf_bcast_->sendTransform(tf_msg);
    }

    void create_camera_info_publisher(
        const ouster::sdk::core::SensorInfo& sensor_info,
        const std::string& frame_id,
        const rclcpp::QoS& qos) {
        uint32_t H = sensor_info.format.pixels_per_column;
        uint32_t W = sensor_info.format.columns_per_frame;

        // Equirectangular projection for the LiDAR range image.
        // Horizontal: each column = 2pi/W radians (full rotation, azimuth
        // window only affects data validity, not the column-to-angle mapping).
        double fx = static_cast<double>(W) / (2.0 * M_PI);
        double cx = static_cast<double>(W) / 2.0;

        // Vertical: use actual beam altitude angles for correct VFOV.
        // The beams are approximately uniformly spaced; per-beam non-uniformity
        // cannot be represented in a single fy but this is far closer than 2pi.
        double fy;
        double cy;
        const auto& alts = sensor_info.beam_altitude_angles;
        double vfov_rad = 0.0;
        double max_alt_rad = 0.0;
        if (alts.size() >= 2) {
            auto [min_it, max_it] = std::minmax_element(alts.begin(), alts.end());
            vfov_rad = (*max_it - *min_it) * M_PI / 180.0;
            max_alt_rad = *max_it * M_PI / 180.0;
        }
        if (vfov_rad > 0.0) {
            // The top and bottom beams sit at rows 0 and H-1, so the measured
            // VFOV spans H-1 pixel pitches, not H.
            fy = static_cast<double>(H - 1) / vfov_rad;
            // cy: row where elevation = 0 deg. Beams are ordered top-to-bottom
            // (highest altitude = row 0), so row(e) = (max_alt - e) * fy and
            // the horizon lands at:
            cy = max_alt_rad * fy;
        } else {
            // Degenerate fallback (fewer than 2 beams, or all altitudes
            // equal): assumes full 2pi VFOV, which is wrong for any real
            // Ouster sensor. Downstream rectification/projection will be
            // garbage. Warn loudly so this isn't debugged in silence.
            RCLCPP_WARN(get_logger(),
                "CameraInfo: beam_altitude_angles has %zu element(s) spanning "
                "%f rad (need >=2 distinct altitudes). Falling back to a "
                "degenerate 2pi-VFOV calibration; K matrix will not match "
                "real sensor geometry. Check sensor metadata.",
                alts.size(), vfov_rad);
            fy = static_cast<double>(H) / (2.0 * M_PI);
            cy = static_cast<double>(H) / 2.0;
        }

        camera_info_msg_.header.frame_id = frame_id;
        camera_info_msg_.height = H;
        camera_info_msg_.width = W;
        camera_info_msg_.distortion_model = get_parameter("distortion_model").as_string();
        camera_info_msg_.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        camera_info_msg_.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        camera_info_msg_.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0,
                              1.0, 0.0};

        // ROI from sensor metadata column_window. Ouster sensors support
        // azimuth windowing (e.g. forward-only sector to suppress backplate
        // self-returns); the published image is still full-width (invalid
        // columns are zero-filled), but ROI tells consumers which columns
        // hold real data. column_window is inclusive on both ends and given
        // in raw (staggered) columns, while the published images are
        // destaggered. The SDK destagger maps a staggered column c to image
        // column (c + pixel_shift_by_row[u]) mod W (see ouster::destagger:
        // destaggered[k] = staggered[k - shift]), so the valid region shifts
        // per row. A CameraInfo ROI is a single rectangle, so publish the
        // bounding box over all rows when it fits without wrapping; otherwise
        // fall back to a full-size ROI.
        const auto& cw = sensor_info.format.column_window;
        const auto& shifts = sensor_info.format.pixel_shift_by_row;
        int min_shift = 0;
        int max_shift = 0;
        if (!shifts.empty()) {
            auto [min_it, max_it] =
                std::minmax_element(shifts.begin(), shifts.end());
            min_shift = *min_it;
            max_shift = *max_it;
        }
        const int x0 = cw.first + min_shift;   // leftmost valid image column
        const int x1 = cw.second + max_shift;  // rightmost valid image column
        if (cw.first <= cw.second && x0 >= 0 && x1 < static_cast<int>(W)) {
            camera_info_msg_.roi.x_offset = static_cast<uint32_t>(x0);
            camera_info_msg_.roi.y_offset = 0;
            camera_info_msg_.roi.width = static_cast<uint32_t>(x1 - x0 + 1);
            camera_info_msg_.roi.height = H;
        } else {
            RCLCPP_INFO(get_logger(),
                "CameraInfo: column_window=[%d,%d] with destagger shifts "
                "[%d,%d] makes the valid region wrap past the image bounds; "
                "publishing a full-size ROI (equivalent to unset).",
                cw.first, cw.second, min_shift, max_shift);
            camera_info_msg_.roi.x_offset = 0;
            camera_info_msg_.roi.y_offset = 0;
            camera_info_msg_.roi.width = W;
            camera_info_msg_.roi.height = H;
        }
        camera_info_msg_.roi.do_rectify = false;

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
    bool publish_camera_info_{true};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_bcast_;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};
}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterImage)
