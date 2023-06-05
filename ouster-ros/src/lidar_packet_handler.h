/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_packet_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl_conversions/pcl_conversions.h>

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

namespace sensor = ouster::sensor;


class LidarPacketHandler {
    using LidarPacketAccumlator = std::function<bool(const uint8_t*)>;


    public:
    using HandlerOutput = std::vector<std::shared_ptr<sensor_msgs::msg::PointCloud2>>;
    using HandlerType = std::function<HandlerOutput(const uint8_t*)>;

   public:
    LidarPacketHandler(const ouster::sensor::sensor_info& info, const std::string& frame,
                       bool apply_lidar_to_sensor_transform, bool use_ros_time) :
        ref_frame(frame) {
        create_lidarscan_objects(info, apply_lidar_to_sensor_transform);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };

        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.mode);
        const sensor::packet_format& pf = sensor::get_format(info);

        lidar_packet_accumlator = use_ros_time ?
            LidarPacketAccumlator{[this, pf](const uint8_t* lidar_buf) {
                return lidar_handler_ros_time(pf, lidar_buf); }} :
            LidarPacketAccumlator{[this, pf](const uint8_t* lidar_buf) {
                return lidar_handler_sensor_time(pf, lidar_buf); }};
    }

    LidarPacketHandler(const LidarPacketHandler&) = delete;
    LidarPacketHandler& operator=(const LidarPacketHandler&) = delete;
    ~LidarPacketHandler() = default;

   public:
    static HandlerType create_handler(
        const ouster::sensor::sensor_info& info, const std::string& frame, bool apply_lidar_to_sensor_transform, bool use_ros_time) {
        auto handler = std::make_shared<LidarPacketHandler>(info, frame, apply_lidar_to_sensor_transform, use_ros_time);
        return [handler](const uint8_t* lidar_buf) {
            return handler->lidar_packet_accumlator(lidar_buf) ? handler->pc_msgs : HandlerOutput{};
        };
    }

    static int num_returns(const ouster::sensor::sensor_info& info) {
        using ouster::sensor::UDPProfileLidar;
        return info.format.udp_profile_lidar ==
                   UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
               ? 2
               : 1;
    }

    private:
    void create_lidarscan_objects(const ouster::sensor::sensor_info& info, bool apply_lidar_to_sensor_transform) {
        ouster::mat4d additional_transform =
            apply_lidar_to_sensor_transform ?
                info.lidar_to_sensor_transform :
                ouster::mat4d::Identity();
        auto xyz_lut = ouster::make_xyz_lut(
                        info.format.columns_per_frame,
                        info.format.pixels_per_column,
                        ouster::sensor::range_unit,
                        additional_transform,
                        ouster::mat4d::Identity(),
                        info.beam_azimuth_angles,
                        info.beam_altitude_angles);
        // The ouster_ros drive currently only uses single precision when it
        // produces the point cloud. So it isn't of a benefit to compute point
        // cloud xyz coordinates using double precision (for the time being).
        lut_direction = xyz_lut.direction.cast<float>();
        lut_offset = xyz_lut.offset.cast<float>();
        points = ouster::PointsF(lut_direction.rows(), lut_offset.cols());
        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;
        lidar_scan = std::make_unique<ouster::LidarScan>(
            W, H, info.format.udp_profile_lidar);
        cloud = ouster_ros::Cloud{W, H};
        pc_msgs.resize(num_returns(info), std::make_shared<sensor_msgs::msg::PointCloud2>());
    }

    void pcl_toROSMsg(const ouster_ros::Cloud& pcl_cloud,
                      sensor_msgs::msg::PointCloud2& cloud) {
        // TODO: remove the staging step in the future
        pcl::toPCLPointCloud2(pcl_cloud, staging_pcl_pc2);
        pcl_conversions::moveFromPCL(staging_pcl_pc2, cloud);
    }

    void convert_scan_to_pointcloud(uint64_t scan_ts,
                                    const rclcpp::Time& msg_ts) {
        for (int i = 0; i < static_cast<int>(pc_msgs.size()); ++i) {
            scan_to_cloud_f(points, lut_direction, lut_offset, scan_ts,
                            *lidar_scan, cloud, i);
            pcl_toROSMsg(cloud, *pc_msgs[i]);
            pc_msgs[i]->header.stamp = msg_ts;
            pc_msgs[i]->header.frame_id = ref_frame;
        }
    }

    // time interpolation methods
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

    uint16_t packet_col_index(const sensor::packet_format& pf, const uint8_t* lidar_buf) {
        return pf.col_measurement_id(pf.nth_col(0, lidar_buf));
    }

    rclcpp::Time extrapolate_frame_ts(const sensor::packet_format& pf,
                                      const uint8_t* lidar_buf,
                                      const rclcpp::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(pf, lidar_buf);
        auto delta_time = rclcpp::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    bool lidar_handler_sensor_time(const sensor::packet_format&, const uint8_t* lidar_buf) {
        if (!(*scan_batcher)(lidar_buf, *lidar_scan)) return false;
        auto scan_ts = compute_scan_ts(lidar_scan->timestamp());
        convert_scan_to_pointcloud(scan_ts, rclcpp::Time(scan_ts));
        return true;
    }

    bool lidar_handler_ros_time(const sensor::packet_format& pf,
                                const uint8_t* lidar_buf) {
        auto packet_receive_time = rclcpp::Clock(RCL_ROS_TIME).now();
        if (!lidar_handler_ros_time_frame_ts_initialized) {
            lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
                pf, lidar_buf, packet_receive_time);  // first point cloud time
            lidar_handler_ros_time_frame_ts_initialized = true;
        }
        if (!(*scan_batcher)(lidar_buf, *lidar_scan)) return false;
        auto scan_ts = compute_scan_ts(lidar_scan->timestamp());
        convert_scan_to_pointcloud(scan_ts, lidar_handler_ros_time_frame_ts);
        // set time for next point cloud msg
        lidar_handler_ros_time_frame_ts =
            extrapolate_frame_ts(pf, lidar_buf, packet_receive_time);
        return true;
    }

    static double compute_scan_col_ts_spacing_ns(sensor::lidar_mode ld_mode) {
        const auto scan_width = sensor::n_cols_of_lidar_mode(ld_mode);
        const auto scan_frequency = sensor::frequency_of_lidar_mode(ld_mode);
        const double one_sec_in_ns = 1e+9;
        return one_sec_in_ns / (scan_width * scan_frequency);
    }

    private:
    std::string ref_frame;

    ouster::PointsF lut_direction;
    ouster::PointsF lut_offset;
    ouster::PointsF points;
    std::unique_ptr<ouster::LidarScan> lidar_scan;
    ouster_ros::Cloud cloud;
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;

    // a buffer used for staging during the conversion
    // from a PCL point cloud to a ros point cloud message
    pcl::PCLPointCloud2 staging_pcl_pc2;

    bool lidar_handler_ros_time_frame_ts_initialized = false;
    rclcpp::Time lidar_handler_ros_time_frame_ts;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;

    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;

    public:
    HandlerOutput pc_msgs;
    LidarPacketAccumlator lidar_packet_accumlator;
};