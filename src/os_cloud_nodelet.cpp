/**
 * Copyright (c) 2018-2023, Ouster, Inc.
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

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <cassert>

#include "ouster_ros/GetMetadata.h"
#include "ouster_ros/PacketMsg.h"

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;
using sensor::UDPProfileLidar;
using namespace std::chrono_literals;

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

template <typename X, typename Y>
Y linear_interpolate(X x0, Y y0, X x1, Y y1, X x) {
    return y0 + (x - x0) * (y1 - y0) / (x1 - x0);
}

template <typename T>
uint64_t ulround(T value) {
    T rounded_value = std::round(value);
    return std::max(static_cast<T>(0),
                    std::min(rounded_value, static_cast<T>(ULLONG_MAX)));
}
}  // namespace

namespace nodelets_os {
class OusterCloud : public nodelet::Nodelet {
   private:
    bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    virtual void onInit() override {
        parse_parameters();
        auto& nh = getNodeHandle();
        auto metadata = get_metadata(nh);
        info = sensor::parse_metadata(metadata);
        n_returns = compute_n_returns(info.format);
        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.mode);
        create_lidarscan_objects();
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };
        create_publishers(nh);
        create_subscribers(nh);
    }

    void parse_parameters() {
        auto& pnh = getPrivateNodeHandle();
        auto tf_prefix = pnh.param("tf_prefix", std::string{});
        if (is_arg_set(tf_prefix) && tf_prefix.back() != '/')
            tf_prefix.append("/");
        sensor_frame = tf_prefix + "os_sensor";
        imu_frame = tf_prefix + "os_imu";
        lidar_frame = tf_prefix + "os_lidar";
        auto timestamp_mode_arg = pnh.param("timestamp_mode", std::string{});
        use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";
    }

    std::string get_metadata(ros::NodeHandle& nh) {
        ouster_ros::GetMetadata request;
        auto client = nh.serviceClient<ouster_ros::GetMetadata>("get_metadata");
        client.waitForExistence();
        if (!client.call(request)) {
            auto error_msg = "OusterCloud: Calling get_metadata service failed";
            NODELET_ERROR_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }

        NODELET_INFO("OusterCloud: retrieved sensor metadata!");
        return request.response.metadata;
    }

    static int compute_n_returns(const sensor::data_format& format) {
        return format.udp_profile_lidar ==
                       UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
                   ? 2
                   : 1;
    }

    static double compute_scan_col_ts_spacing_ns(sensor::lidar_mode ld_mode) {
        const auto scan_width = sensor::n_cols_of_lidar_mode(ld_mode);
        const auto scan_frequency = sensor::frequency_of_lidar_mode(ld_mode);
        const double one_sec_in_ns = 1e+9;
        return one_sec_in_ns / (scan_width * scan_frequency);
    }

    void create_lidarscan_objects() {
        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        // cloud xyz coordinates using double precision (for the time being).
        auto xyz_lut = ouster::make_xyz_lut(info);
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(lut_direction.rows(), lut_offset.cols());
        pc_ptr = boost::make_shared<sensor_msgs::PointCloud2>();

        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        ls = ouster::LidarScan{W, H, info.format.udp_profile_lidar};
        cloud = ouster_ros::Cloud{W, H};

        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);
    }

    void create_publishers(ros::NodeHandle& nh) {
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);

        auto img_suffix = [](int ind) {
            if (ind == 0) return std::string();
            return std::to_string(ind + 1);  // need second return to return 2
        };

        lidar_pubs.resize(n_returns);
        for (int i = 0; i < n_returns; i++) {
            auto pub = nh.advertise<sensor_msgs::PointCloud2>(
                std::string("points") + img_suffix(i), 10);
            lidar_pubs[i] = pub;
        }
    }

    void create_subscribers(ros::NodeHandle& nh) {
        auto lidar_handler = use_ros_time
                                 ? &OusterCloud::lidar_handler_ros_time
                                 : &OusterCloud::lidar_handler_sensor_time;

        lidar_packet_sub =
            nh.subscribe<PacketMsg>("lidar_packets", 2048, lidar_handler, this);
        imu_packet_sub = nh.subscribe<PacketMsg>(
            "imu_packets", 100, &OusterCloud::imu_handler, this);
    }

    void pcl_toROSMsg(const ouster_ros::Cloud& pcl_cloud,
                      sensor_msgs::PointCloud2& cloud) {
        // TODO: remove the staging step in the future
        static pcl::PCLPointCloud2 pcl_pc2;
        pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
        pcl_conversions::moveFromPCL(pcl_pc2, cloud);
    }

    void convert_scan_to_pointcloud_publish(std::chrono::nanoseconds scan_ts,
                                            const ros::Time& msg_ts) {
        for (int i = 0; i < n_returns; ++i) {
            scan_to_cloud_f(points, lut_direction, lut_offset, scan_ts, ls,
                            cloud, i);
            pcl_toROSMsg(cloud, *pc_ptr);
            pc_ptr->header.stamp = msg_ts;
            pc_ptr->header.frame_id = sensor_frame;
            lidar_pubs[i].publish(pc_ptr);
        }

        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.lidar_to_sensor_transform, sensor_frame, lidar_frame, msg_ts));
    }

    uint64_t impute_value(int last_scan_last_nonzero_idx,
                          uint64_t last_scan_last_nonzero_value,
                          int curr_scan_first_nonzero_idx,
                          uint64_t curr_scan_first_nonzero_value,
                          int scan_width) {
        assert(scan_width + curr_scan_first_nonzero_idx >
               last_scan_last_nonzero_idx);
        double interpolated_value = linear_interpolate(
            last_scan_last_nonzero_idx,
            static_cast<double>(last_scan_last_nonzero_value),
            scan_width + curr_scan_first_nonzero_idx,
            static_cast<double>(curr_scan_first_nonzero_value), scan_width);
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
    std::chrono::nanoseconds compute_scan_ts_0(
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
        return std::chrono::nanoseconds(scan_ns);
    }

    // compute_scan_ts_n applied to all subsequent scans except first one
    std::chrono::nanoseconds compute_scan_ts_n(
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
        return std::chrono::nanoseconds(scan_ns);
    }

    void lidar_handler_sensor_time(const PacketMsg::ConstPtr& packet) {
        if (!(*scan_batcher)(packet->buf.data(), ls)) return;
        auto scan_ts = compute_scan_ts(ls.timestamp());
        convert_scan_to_pointcloud_publish(scan_ts, to_ros_time(scan_ts));
    }

    uint16_t packet_col_index(const uint8_t* packet_buf) {
        const auto& pf = sensor::get_format(info);
        return pf.col_measurement_id(pf.nth_col(0, packet_buf));
    }

    ros::Time extrapolate_frame_ts(const uint8_t* lidar_buf,
                                   const ros::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(lidar_buf);
        auto delta_time = ros::Duration(
            0, scan_col_ts_spacing_ns * curr_scan_first_arrived_idx);
        return current_time - delta_time;
    }

    void lidar_handler_ros_time(const PacketMsg::ConstPtr& packet) {
        auto packet_receive_time = ros::Time::now();
        const uint8_t* packet_buf = packet->buf.data();
        static auto frame_ts = extrapolate_frame_ts(
            packet_buf, packet_receive_time);  // first point cloud time
        if (!(*scan_batcher)(packet_buf, ls)) return;
        auto scan_ts = compute_scan_ts(ls.timestamp());
        convert_scan_to_pointcloud_publish(scan_ts, frame_ts);
        frame_ts = extrapolate_frame_ts(
            packet_buf,
            packet_receive_time);  // set time for next point cloud msg
    }

    void imu_handler(const PacketMsg::ConstPtr& packet) {
        const auto& pf = sensor::get_format(info);
        ros::Time msg_ts =
            use_ros_time ? ros::Time::now()
                         : to_ros_time(pf.imu_gyro_ts(packet->buf.data()));
        sensor_msgs::Imu imu_msg =
            ouster_ros::packet_to_imu_msg(*packet, msg_ts, imu_frame, pf);
        sensor_msgs::ImuPtr imu_msg_ptr =
            boost::make_shared<sensor_msgs::Imu>(imu_msg);
        imu_pub.publish(imu_msg_ptr);

        tf_bcast.sendTransform(ouster_ros::transform_to_tf_msg(
            info.imu_to_sensor_transform, sensor_frame, imu_frame, msg_ts));
    };

    inline ros::Time to_ros_time(uint64_t ts) {
        ros::Time t;
        t.fromNSec(ts);
        return t;
    }

    inline ros::Time to_ros_time(std::chrono::nanoseconds ts) {
        return to_ros_time(ts.count());
    }

   private:
    ros::Subscriber lidar_packet_sub;
    std::vector<ros::Publisher> lidar_pubs;
    ros::Subscriber imu_packet_sub;
    ros::Publisher imu_pub;
    sensor_msgs::PointCloud2::Ptr pc_ptr;

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

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;
    std::function<std::chrono::nanoseconds(
        const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;
    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan
};

}  // namespace nodelets_os

PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterCloud, nodelet::Nodelet)
