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

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

#include "os_static_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster_msgs::msg::PacketMsg;
using sensor::UDPProfileLidar;

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
        declare_parameter("point_type", "default");
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterCloud: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        tf_bcast.broadcast_transforms(info);
        create_publishers_subscriptions(info);
    }

    template <typename PointT>
    LidarScanProcessor make_point_cloud_procssor(const sensor::sensor_info& info) {
        return PointCloudProcessor<PointT>::create(
            info, tf_bcast.point_cloud_frame_id(),
            tf_bcast.apply_lidar_to_sensor_transform(),
            [this](typename PointCloudProcessor<PointT>::OutputType msgs) {
                for (size_t i = 0; i < msgs.size(); ++i)
                    lidar_pubs[i]->publish(*msgs[i]);
        });
    }

    LidarScanProcessor create_point_cloud_processor(const std::string& point_type,
                                                    const sensor::sensor_info& info) {
        if (point_type == "xyz") {
            return make_point_cloud_procssor<pcl::PointXYZ>(info);
        } else if (point_type == "xyzi") {
            if (info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8)
                RCLCPP_WARN_STREAM(get_logger(),
                    "selected point type 'xyzi' is not compatible with the current udp profile: RNG15_RFL8_NIR8");
            return make_point_cloud_procssor<pcl::PointXYZ>(info);
        } else if (point_type == "xyzir") {
            if (info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8)
                RCLCPP_WARN_STREAM(get_logger(),
                    "selected point type 'xyzir' is not compatible with the current udp profile: RNG15_RFL8_NIR8");
            return make_point_cloud_procssor<ouster_ros::PointXYZIR>(info);
        } else if (point_type == "auto") {
            switch (info.format.udp_profile_lidar) {
                case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL:
                    return make_point_cloud_procssor<ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_DUAL>(info);
                case UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16:
                    return make_point_cloud_procssor<ouster_ros::Point_RNG19_RFL8_SIG16_NIR16>(info);
                case UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8:
                    return make_point_cloud_procssor<ouster_ros::Point_RNG15_RFL8_NIR8>(info);
                // case PROFILE_LIDAR_LEGACY:   Should we define legacy?
                default:
                    RCLCPP_WARN_STREAM(get_logger(),
                        "point_type is set to auto but current udp_profile_lidar is unsupported "
                        "! falling back to the driver default/legacy pcl point format");
            }
        } else if (point_type != "legacy") {
            RCLCPP_WARN_STREAM(get_logger(),
                "Un-supported point type used: " << point_type <<
                "! falling back to driver default/legacy pcl point format");
        }

        return make_point_cloud_procssor<ouster_ros::Point>(info);
    }

    void create_publishers_subscriptions(const sensor::sensor_info& info) {
        auto timestamp_mode = get_parameter("timestamp_mode").as_string();
        auto ptp_utc_tai_offset =
            get_parameter("ptp_utc_tai_offset").as_double();

        bool use_system_default_qos =
            get_parameter("use_system_default_qos").as_bool();
        rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
        rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
        auto selected_qos =
            use_system_default_qos ? system_default_qos : sensor_data_qos;

        auto proc_mask = get_parameter("proc_mask").as_string();
        auto tokens = impl::parse_tokens(proc_mask, '|');

        if (impl::check_token(tokens, "IMU")) {
            imu_pub =
                create_publisher<sensor_msgs::msg::Imu>("imu", selected_qos);
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
            imu_packet_sub = create_subscription<PacketMsg>(
                "imu_packets", selected_qos,
                [this](const PacketMsg::ConstSharedPtr msg) {
                    auto imu_msg = imu_packet_handler(msg->buf.data());
                    imu_pub->publish(imu_msg);
                });
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
            processors.push_back(create_point_cloud_processor(point_type, info));
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
                    for (size_t i = 0; i < msgs.size(); ++i)
                        scan_pubs[i]->publish(*msgs[i]);
                }));
        }

        if (impl::check_token(tokens, "PCL") || impl::check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
            lidar_packet_sub = create_subscription<PacketMsg>(
                "lidar_packets", selected_qos,
                [this](const PacketMsg::ConstSharedPtr msg) {
                    lidar_packet_handler(msg->buf.data());
                });
        }
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
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
