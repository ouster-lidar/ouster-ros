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
#include <tf2_ros/static_transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <cassert>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_srvs/srv/get_metadata.hpp"
#include "ouster_ros/os_processing_node_base.h"
#include "ouster_ros/visibility_control.h"

namespace sensor = ouster::sensor;
using ouster_msgs::msg::PacketMsg;

namespace {

template <typename T, typename UnaryPredicate>
int find_if_reverse(const Eigen::Array<T, -1, 1>& array,
                    UnaryPredicate predicate) {
    auto p = array.data() + array.size() - 1;
    do {
        if (predicate(*p)) return p - array.data();
    } while (p-- != array.data());
    return -1;
}

uint64_t linear_interpolate(int x0, uint64_t y0, int x1, uint64_t y1, int x) {
    uint64_t min_v, max_v;
    double sign;
    if (y1 > y0) {
        min_v = y0;
        max_v = y1;
        sign = +1;
    } else {
        min_v = y1;
        max_v = y0;
        sign = -1;
    }
    return y0 + (x - x0) * sign * (max_v - min_v) / (x1 - x0);
}

template <typename T>
uint64_t ulround(T value) {
    T rounded_value = std::round(value);
    if (rounded_value < 0) return 0ULL;
    if (rounded_value > ULLONG_MAX) return ULLONG_MAX;
    return static_cast<uint64_t>(rounded_value);
}

}  // namespace

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
        create_metadata_subscriber(
            [this](const auto& msg) { metadata_handler(msg); });
        RCLCPP_INFO(get_logger(), "OusterCloud: node initialized!");
    }

    void declare_parameters() {
        declare_parameter<std::string>("sensor_frame");
        declare_parameter<std::string>("lidar_frame");
        declare_parameter<std::string>("imu_frame");
        declare_parameter<std::string>("timestamp_mode");
    }

    void parse_parameters() {
        sensor_frame = get_parameter("sensor_frame").as_string();
        lidar_frame = get_parameter("lidar_frame").as_string();
        imu_frame = get_parameter("imu_frame").as_string();

        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";
    }

    static double compute_scan_col_ts_spacing_ns(sensor::lidar_mode ld_mode) {
        const auto scan_width = sensor::n_cols_of_lidar_mode(ld_mode);
        const auto scan_frequency = sensor::frequency_of_lidar_mode(ld_mode);
        const double one_sec_in_ns = 1e+9;
        return one_sec_in_ns / (scan_width * scan_frequency);
    }

    void metadata_handler(
        const std_msgs::msg::String::ConstSharedPtr& metadata_msg) {
        RCLCPP_INFO(get_logger(),
                    "OusterCloud: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        send_static_transforms();
        n_returns = get_n_returns();
        create_lidarscan_objects();
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };
        create_publishers();
        create_subscriptions();
    }

    void send_static_transforms() {
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, now()));
        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, now()));
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
        pcl::toPCLPointCloud2(pcl_cloud, staging_pcl_pc2);
        pcl_conversions::moveFromPCL(staging_pcl_pc2, cloud);
    }

    void convert_scan_to_pointcloud_publish(uint64_t scan_ts,
                                            const rclcpp::Time& msg_ts) {
        for (int i = 0; i < n_returns; ++i) {
            scan_to_cloud_f(points, lut_direction, lut_offset, scan_ts,
                            *lidar_scan, cloud, i);
            pcl_toROSMsg(cloud, pc_msg);
            pc_msg.header.stamp = msg_ts;
            pc_msg.header.frame_id = sensor_frame;
            lidar_pubs[i]->publish(pc_msg);
        }
    }

    uint64_t impute_value(int last_scan_last_nonzero_idx,
                          uint64_t last_scan_last_nonzero_value,
                          int curr_scan_first_nonzero_idx,
                          uint64_t curr_scan_first_nonzero_value,
                          int scan_width) {
        assert(scan_width + curr_scan_first_nonzero_idx >
               last_scan_last_nonzero_idx);
        double interpolated_value = linear_interpolate(
            last_scan_last_nonzero_idx, last_scan_last_nonzero_value,
            scan_width + curr_scan_first_nonzero_idx,
            curr_scan_first_nonzero_value, scan_width);
        return ulround(interpolated_value);
    }

    uint64_t extrapolate_value(int curr_scan_first_nonzero_idx,
                               uint64_t curr_scan_first_nonzero_value) {
        double extrapolated_value =
            curr_scan_first_nonzero_value -
            scan_col_ts_spacing_ns * curr_scan_first_nonzero_idx;
        return ulround(extrapolated_value);
    }

    // compute_scan_ts_0 for first scan
    uint64_t compute_scan_ts_0(
        const ouster::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        assert(idx != ts_v.data() + ts_v.size());  // should never happen
        int curr_scan_first_nonzero_idx = idx - ts_v.data();
        uint64_t curr_scan_first_nonzero_value = *idx;

        uint64_t scan_ns =
            curr_scan_first_nonzero_idx == 0
                ? curr_scan_first_nonzero_value
                : extrapolate_value(curr_scan_first_nonzero_idx,
                                    curr_scan_first_nonzero_value);

        last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
        assert(last_scan_last_nonzero_idx >= 0);  // should never happen
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_n(ts_v);
        };
        return scan_ns;
    }

    // compute_scan_ts_n applied to all subsequent scans except first one
    uint64_t compute_scan_ts_n(
        const ouster::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        assert(idx != ts_v.data() + ts_v.size());  // should never happen
        int curr_scan_first_nonzero_idx = idx - ts_v.data();
        uint64_t curr_scan_first_nonzero_value = *idx;

        uint64_t scan_ns = curr_scan_first_nonzero_idx == 0
                               ? curr_scan_first_nonzero_value
                               : impute_value(last_scan_last_nonzero_idx,
                                              last_scan_last_nonzero_value,
                                              curr_scan_first_nonzero_idx,
                                              curr_scan_first_nonzero_value,
                                              static_cast<int>(ts_v.size()));

        last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
        assert(last_scan_last_nonzero_idx >= 0);  // should never happen
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        return scan_ns;
    }

    void lidar_handler_sensor_time(const PacketMsg::ConstPtr& packet) {
        if (!(*scan_batcher)(packet->buf.data(), *lidar_scan)) return;
        auto scan_ts = compute_scan_ts(lidar_scan->timestamp());
        convert_scan_to_pointcloud_publish(scan_ts, to_ros_time(scan_ts));
    }

    uint16_t packet_col_index(const uint8_t* packet_buf) {
        const auto& pf = sensor::get_format(info);
        return pf.col_measurement_id(pf.nth_col(0, packet_buf));
    }

    rclcpp::Time extrapolate_frame_ts(const uint8_t* lidar_buf,
                                      const rclcpp::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(lidar_buf);
        auto delta_time = rclcpp::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    void lidar_handler_ros_time(const PacketMsg::ConstPtr& packet) {
        auto packet_receive_time = rclcpp::Clock(RCL_ROS_TIME).now();
        const uint8_t* packet_buf = packet->buf.data();
        if (!lidar_handler_ros_time_frame_ts_initialized) {
            lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
                packet_buf, packet_receive_time);  // first point cloud time
            lidar_handler_ros_time_frame_ts_initialized = true;
        }
        if (!(*scan_batcher)(packet_buf, *lidar_scan)) return;
        auto scan_ts = compute_scan_ts(lidar_scan->timestamp());
        convert_scan_to_pointcloud_publish(scan_ts,
                                           lidar_handler_ros_time_frame_ts);
        // set time for next point cloud msg
        lidar_handler_ros_time_frame_ts =
            extrapolate_frame_ts(packet_buf, packet_receive_time);
    }

    void imu_handler(const PacketMsg::ConstSharedPtr packet) {
        auto pf = sensor::get_format(info);
        auto msg_ts = use_ros_time
                          ? rclcpp::Clock(RCL_ROS_TIME).now()
                          : to_ros_time(pf.imu_gyro_ts(packet->buf.data()));
        auto imu_msg =
            ouster_ros::packet_to_imu_msg(*packet, msg_ts, imu_frame, pf);
        imu_pub->publish(imu_msg);
    };

    static inline rclcpp::Time to_ros_time(uint64_t ts) {
        return rclcpp::Time(ts);
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

    tf2_ros::StaticTransformBroadcaster tf_bcast;

    bool use_ros_time;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;
    std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;
    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    pcl::PCLPointCloud2
        staging_pcl_pc2;  // a buffer used for staging during the conversion
                          // from a PCL point cloud to a ros point cloud message

    bool lidar_handler_ros_time_frame_ts_initialized = false;
    rclcpp::Time lidar_handler_ros_time_frame_ts;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterCloud)
