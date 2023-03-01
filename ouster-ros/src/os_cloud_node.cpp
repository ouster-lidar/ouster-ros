/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_node.cpp
 * @brief A node to publish point clouds and imu topics
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <memory>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_srvs/srv/get_metadata.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

namespace sensor = ouster::sensor;
using ouster_msgs::msg::PacketMsg;
using ouster_srvs::srv::GetMetadata;

namespace ouster_ros {

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
        parse_parameters();
        auto metadata = get_metadata();
        info = sensor::parse_metadata(metadata);
        n_returns = get_n_returns();
        create_lidarscan_objects();
        create_publishers();
        create_subscriptions();
    }

    void declare_parameters() {
        declare_parameter("tf_prefix");
        declare_parameter("timestamp_mode");
    }

    void parse_parameters() {
        auto tf_prefix = get_parameter("tf_prefix").as_string();
        if (is_arg_set(tf_prefix) && tf_prefix.back() != '/')
            tf_prefix.append("/");
        sensor_frame = tf_prefix + "os_sensor";
        imu_frame = tf_prefix + "os_imu";
        lidar_frame = tf_prefix + "os_lidar";
        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";
    }

    void create_lidarscan_objects() {
        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        // cloud xyz coordinates using double precision (for the time being).
        auto xyz_lut = ouster::make_xyz_lut(info);
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(lut_direction.rows(), lut_offset.cols());
        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        lidar_scan = std::make_unique<ouster::LidarScan>(
            W, H, info.format.udp_profile_lidar);
        cloud = ouster_ros::Cloud{W, H};
    }

    void create_publishers() {
        rclcpp::SensorDataQoS qos;
        imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", qos);
        lidar_pubs.resize(n_returns);
        for (int i = 0; i < n_returns; i++) {
            lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                topic_for_return("points", i), qos);
        }
    }

    void create_subscriptions() {
        rclcpp::SensorDataQoS qos;

        using LidarHandlerFunctionType =
            std::function<void(const PacketMsg::ConstSharedPtr)>;
        LidarHandlerFunctionType lidar_handler_ros_time_function =
            [this](const PacketMsg::ConstSharedPtr msg) {
                lidar_handler_ros_time(msg);
            };
        LidarHandlerFunctionType lidar_handler_sensor_time_function =
            [this](const PacketMsg::ConstSharedPtr msg) {
                lidar_handler_sensor_time(msg);
            };
        auto lidar_handler = use_ros_time ? lidar_handler_ros_time_function
                                          : lidar_handler_sensor_time_function;
        lidar_packet_sub = create_subscription<PacketMsg>(
            "lidar_packets", qos,
            [this, lidar_handler](const PacketMsg::ConstSharedPtr msg) {
                lidar_handler(msg);
            });
        imu_packet_sub = create_subscription<PacketMsg>(
            "imu_packets", qos,
            [this](const PacketMsg::ConstSharedPtr msg) { imu_handler(msg); });
    }

    void pcl_toROSMsg(const ouster_ros::Cloud& pcl_cloud,
                      sensor_msgs::msg::PointCloud2& cloud) {
        // TODO: remove the staging step in the future
        static pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
        pcl_conversions::moveFromPCL(pcl_pc2, cloud);
    }

    void convert_scan_to_pointcloud_publish(std::chrono::nanoseconds scan_ts,
                                            const rclcpp::Time& msg_ts) {
        for (int i = 0; i < n_returns; ++i) {
            scan_to_cloud_f(points, lut_direction, lut_offset, scan_ts,
                            *lidar_scan, cloud, i);
            pcl_toROSMsg(cloud, pc_msg);
            pc_msg.header.stamp = msg_ts;
            pc_msg.header.frame_id = sensor_frame;
            lidar_pubs[i]->publish(pc_msg);
        }

        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, msg_ts));
    }

    void lidar_handler_sensor_time(const PacketMsg::ConstSharedPtr packet) {
        if (!(*scan_batcher)(packet->buf.data(), *lidar_scan)) return;
        auto ts_v = lidar_scan->timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) return;
        auto scan_ts = std::chrono::nanoseconds{ts_v(idx - ts_v.data())};
        convert_scan_to_pointcloud_publish(scan_ts, to_ros_time(scan_ts));
    }

    void lidar_handler_ros_time(const PacketMsg::ConstSharedPtr packet) {
        auto packet_receive_time = get_clock()->now();
        static auto frame_ts = packet_receive_time;  // first point cloud time
        if (!(*scan_batcher)(packet->buf.data(), *lidar_scan)) return;
        auto ts_v = lidar_scan->timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) return;
        auto scan_ts = std::chrono::nanoseconds{ts_v(idx - ts_v.data())};
        convert_scan_to_pointcloud_publish(scan_ts, frame_ts);
        frame_ts = packet_receive_time;  // set time for next point cloud msg
    }

    void imu_handler(const PacketMsg::ConstSharedPtr packet) {
        auto pf = sensor::get_format(info);
        auto msg_ts = use_ros_time
                          ? rclcpp::Clock(RCL_ROS_TIME).now()
                          : to_ros_time(pf.imu_gyro_ts(packet->buf.data()));
        auto imu_msg =
            ouster_ros::packet_to_imu_msg(*packet, msg_ts, imu_frame, pf);
        imu_pub->publish(imu_msg);
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, msg_ts));
    };

    static inline rclcpp::Time to_ros_time(uint64_t ts) {
        return rclcpp::Time(ts);
    }

    static inline rclcpp::Time to_ros_time(std::chrono::nanoseconds ts) {
        return to_ros_time(ts.count());
    }

   private:
    rclcpp::Subscription<PacketMsg>::SharedPtr lidar_packet_sub;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr>
        lidar_pubs;
    rclcpp::Subscription<PacketMsg>::SharedPtr imu_packet_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    sensor_msgs::msg::PointCloud2 pc_msg;

    int n_returns = 0;

    ouster::PointsF lut_direction;
    ouster::PointsF lut_offset;
    ouster::PointsF points;
    std::unique_ptr<ouster::LidarScan> lidar_scan;
    ouster_ros::Cloud cloud;
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;

    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;

    tf2_ros::TransformBroadcaster tf_bcast;

    bool use_ros_time;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
