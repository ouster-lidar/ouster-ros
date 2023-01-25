/**
 * Copyright (c) 2018-2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_nodelet.cpp
 * @brief A nodelet to publish point clouds and imu topics
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
#include "ouster_ros/visibility_control.h"
#include "ouster_ros/srv/get_metadata.hpp"

namespace sensor = ouster::sensor;
using sensor::UDPProfileLidar;
using ouster_msgs::msg::PacketMsg;
using ouster_ros::srv::GetMetadata;

namespace nodelets_os {

class OusterCloud : public rclcpp::Node {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterCloud(const rclcpp::NodeOptions& options)
    : rclcpp::Node("os_cloud", options),
      tf_bcast(this) {
        onInit();
    }

   private:
    bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void onInit() {
        declare_parameters();
        parse_parameters();
        auto metadata = get_metadata();
        info = sensor::parse_metadata(metadata);

        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        n_returns = info.format.udp_profile_lidar ==
                            UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
                        ? 2
                        : 1;

        RCLCPP_INFO_STREAM(get_logger(), "Profile has " << n_returns << " return(s)");

        imu_pub = create_publisher<sensor_msgs::msg::Imu>("imu", 100);

        auto img_suffix = [](int ind) {
            if (ind == 0) return std::string();
            return std::to_string(ind + 1);  // need second return to return 2
        };

        lidar_pubs.resize(n_returns);
        for (int i = 0; i < n_returns; i++) {
            lidar_pubs[i] = create_publisher<sensor_msgs::msg::PointCloud2>(
                std::string("points") + img_suffix(i), 10);
        }

        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        //  cloud xyz coordinates using double precision (for the time being).
        auto xyz_lut = ouster::make_xyz_lut(info);
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(lut_direction.rows(), lut_offset.cols());

        ls = ouster::LidarScan{W, H, info.format.udp_profile_lidar};
        cloud = ouster_ros::Cloud{W, H};

        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);

        using LidarHandlerFunctionType = std::function<void(const PacketMsg::ConstSharedPtr)>;
        LidarHandlerFunctionType lidar_handler_ros_time_function =
            [this](const PacketMsg::ConstSharedPtr msg) { lidar_handler_ros_time(msg); };
        LidarHandlerFunctionType lidar_handler_sensor_time_function =
            [this](const PacketMsg::ConstSharedPtr msg) { lidar_handler_sensor_time(msg); };
        auto lidar_handler = use_ros_time
            ? lidar_handler_ros_time_function
            : lidar_handler_sensor_time_function;
        lidar_packet_sub =
            create_subscription<PacketMsg>("lidar_packets", 2048,
                [this, lidar_handler](const PacketMsg::ConstSharedPtr msg) {
                    lidar_handler(msg);
                });
        imu_packet_sub = create_subscription<PacketMsg>("imu_packets", 100,
            [this](const PacketMsg::ConstSharedPtr msg) { imu_handler(msg); });
    }

    void declare_parameters() {
        declare_parameter("tf_prefix", rclcpp::PARAMETER_STRING);
        declare_parameter("timestamp_mode", rclcpp::PARAMETER_STRING);
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

    std::string get_metadata() {
        using namespace std::chrono_literals;
        auto client = create_client<GetMetadata>("get_metadata");
        client->wait_for_service(1s);
        auto request = std::make_shared<GetMetadata::Request>();
        auto result = client->async_send_request(request);
        RCLCPP_INFO(get_logger(), "sent async request!");
        if (rclcpp::spin_until_future_complete(get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS) {
            auto error_msg = "Calling get_metadata service failed";
            RCLCPP_ERROR_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }

        RCLCPP_INFO(get_logger(), "retrieved sensor metadata!");
        return result.get()->metadata;
    }

    void pcl_toROSMsg(const ouster_ros::Cloud &pcl_cloud, sensor_msgs::msg::PointCloud2 &cloud) {
        // TODO: remove the staging step in the future
        static pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
        pcl_conversions::moveFromPCL(pcl_pc2, cloud);
    }

    void convert_scan_to_pointcloud_publish(std::chrono::nanoseconds scan_ts,
                                            const rclcpp::Time& msg_ts) {
        for (int i = 0; i < n_returns; ++i) {
            scan_to_cloud_f(points, lut_direction, lut_offset, scan_ts, ls, cloud, i);
            pcl_toROSMsg(cloud, pc_msg);
            pc_msg.header.stamp = msg_ts;
            pc_msg.header.frame_id = sensor_frame;
            lidar_pubs[i]->publish(pc_msg);
        }

        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, msg_ts));
    }

    void lidar_handler_sensor_time(const PacketMsg::ConstSharedPtr packet) {
        if (!(*scan_batcher)(packet->buf.data(), ls)) return;
        auto ts_v = ls.timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) return;
        auto scan_ts = std::chrono::nanoseconds{ts_v(idx - ts_v.data())};
        convert_scan_to_pointcloud_publish(scan_ts, to_ros_time(scan_ts));
    }

    void lidar_handler_ros_time(const PacketMsg::ConstSharedPtr packet) {
        auto packet_receive_time = get_clock()->now();
        static auto frame_ts = packet_receive_time;  // first point cloud time
        if (!(*scan_batcher)(packet->buf.data(), ls)) return;
        auto ts_v = ls.timestamp();
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) return;
        auto scan_ts = std::chrono::nanoseconds{ts_v(idx - ts_v.data())};
        convert_scan_to_pointcloud_publish(scan_ts, frame_ts);
        frame_ts = packet_receive_time;  // set time for next point cloud msg
    }

    void imu_handler(const PacketMsg::ConstSharedPtr packet) {
        auto pf = sensor::get_format(info);
        auto msg_ts =
            use_ros_time ? rclcpp::Clock(RCL_ROS_TIME).now()
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
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> lidar_pubs;
    rclcpp::Subscription<PacketMsg>::SharedPtr imu_packet_sub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    sensor_msgs::msg::PointCloud2 pc_msg;


    sensor::sensor_info info;
    int n_returns = 0;

    ouster::PointsF lut_direction;
    ouster::PointsF lut_offset;
    ouster::PointsF points;
    ouster::LidarScan ls;
    ouster_ros::Cloud cloud;
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;

    std::string sensor_frame;
    std::string imu_frame;
    std::string lidar_frame;

    tf2_ros::TransformBroadcaster tf_bcast;

    bool use_ros_time;
};

}  // namespace nodelets_os

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(nodelets_os::OusterCloud)
