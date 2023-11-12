/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file lidar_packet_handler.h
 * @brief ...
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
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

// TODO: move to a separate file
std::set<std::string> parse_tokens(const std::string& input, char delim) {
    std::set<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (getline(ss, token, delim)) {
        // Remove leading and trailing whitespaces from the token
        size_t start = token.find_first_not_of(" ");
        size_t end = token.find_last_not_of(" ");
        token = token.substr(start, end - start + 1);
        if (!token.empty()) tokens.insert(token);
    }

    return tokens;
}

inline bool check_token(const std::set<std::string>& tokens,
                        const std::string& token) {
    return tokens.find(token) != tokens.end();
}

}  // namespace

namespace ouster_ros {

namespace sensor = ouster::sensor;

using LidarScanProcessor =
    std::function<void(const ouster::LidarScan&, uint64_t, const ros::Time&)>;

class LidarPacketHandler {
    using LidarPacketAccumlator = std::function<bool(const uint8_t*)>;

   public:
    using HandlerOutput = ouster::LidarScan;

    using HandlerType = std::function<void(const uint8_t*)>;

   public:
    LidarPacketHandler(const ouster::sensor::sensor_info& info,
                       const std::vector<LidarScanProcessor>& handlers,
                       const std::string& timestamp_mode,
                       int64_t ptp_utc_tai_offset)
        : lidar_scan_handlers{handlers} {
        // initialize lidar_scan processor and buffer
        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);
        lidar_scan = std::make_unique<ouster::LidarScan>(
            info.format.columns_per_frame, info.format.pixels_per_column,
            info.format.udp_profile_lidar);

        // initalize time handlers
        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.mode);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };
        const sensor::packet_format& pf = sensor::get_format(info);

        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            lidar_packet_accumlator =
                LidarPacketAccumlator{[this, pf](const uint8_t* lidar_buf) {
                    return lidar_handler_ros_time(pf, lidar_buf);
                }};
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            lidar_packet_accumlator = LidarPacketAccumlator{
                [this, pf, ptp_utc_tai_offset](const uint8_t* lidar_buf) {
                    return lidar_handler_sensor_time_ptp(pf, lidar_buf,
                                                         ptp_utc_tai_offset);
                }};
        } else {
            lidar_packet_accumlator =
                LidarPacketAccumlator{[this, pf](const uint8_t* lidar_buf) {
                    return lidar_handler_sensor_time(pf, lidar_buf);
                }};
        }
    }

    LidarPacketHandler(const LidarPacketHandler&) = delete;
    LidarPacketHandler& operator=(const LidarPacketHandler&) = delete;
    ~LidarPacketHandler() = default;

    void register_lidar_scan_handler(LidarScanProcessor handler) {
        lidar_scan_handlers.push_back(handler);
    }

    void clear_registered_lidar_scan_handlers() { lidar_scan_handlers.clear(); }

   public:
    static HandlerType create_handler(
        const ouster::sensor::sensor_info& info,
        const std::vector<LidarScanProcessor>& handlers,
        const std::string& timestamp_mode, int64_t ptp_utc_tai_offset) {
        auto handler = std::make_shared<LidarPacketHandler>(
            info, handlers, timestamp_mode, ptp_utc_tai_offset);
        return [handler](const uint8_t* lidar_buf) {
            if (handler->lidar_packet_accumlator(lidar_buf)) {
                for (auto h : handler->lidar_scan_handlers) {
                    h(*handler->lidar_scan, handler->lidar_scan_estimated_ts,
                      handler->lidar_scan_estimated_msg_ts);
                }
            }
        };
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

    uint16_t packet_col_index(const sensor::packet_format& pf,
                              const uint8_t* lidar_buf) {
        return pf.col_measurement_id(pf.nth_col(0, lidar_buf));
    }

    ros::Time extrapolate_frame_ts(const sensor::packet_format& pf,
                                   const uint8_t* lidar_buf,
                                   const ros::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(pf, lidar_buf);
        auto delta_time = ros::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    bool lidar_handler_sensor_time(const sensor::packet_format&,
                                   const uint8_t* lidar_buf) {
        if (!(*scan_batcher)(lidar_buf, *lidar_scan)) return false;
        lidar_scan_estimated_ts = compute_scan_ts(lidar_scan->timestamp());
        lidar_scan_estimated_msg_ts =
            impl::ts_to_ros_time(lidar_scan_estimated_ts);
        return true;
    }

    bool lidar_handler_sensor_time_ptp(const sensor::packet_format&,
                                       const uint8_t* lidar_buf,
                                       int64_t ptp_utc_tai_offset) {
        if (!(*scan_batcher)(lidar_buf, *lidar_scan)) return false;
        auto ts_v = lidar_scan->timestamp();
        for (int i = 0; i < ts_v.rows(); ++i)
            ts_v[i] = impl::ts_safe_offset_add(ts_v[i], ptp_utc_tai_offset);
        lidar_scan_estimated_ts = compute_scan_ts(ts_v);
        lidar_scan_estimated_msg_ts =
            impl::ts_to_ros_time(lidar_scan_estimated_ts);
        return true;
    }

    bool lidar_handler_ros_time(const sensor::packet_format& pf,
                                const uint8_t* lidar_buf) {
        auto packet_receive_time = ros::Time::now();
        if (!lidar_handler_ros_time_frame_ts_initialized) {
            lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
                pf, lidar_buf, packet_receive_time);  // first point cloud time
            lidar_handler_ros_time_frame_ts_initialized = true;
        }
        if (!(*scan_batcher)(lidar_buf, *lidar_scan)) return false;
        lidar_scan_estimated_ts = compute_scan_ts(lidar_scan->timestamp());
        lidar_scan_estimated_msg_ts = lidar_handler_ros_time_frame_ts;
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
    std::unique_ptr<ouster::ScanBatcher> scan_batcher;
    std::unique_ptr<ouster::LidarScan> lidar_scan;
    uint64_t lidar_scan_estimated_ts;
    ros::Time lidar_scan_estimated_msg_ts;

    bool lidar_handler_ros_time_frame_ts_initialized = false;
    ros::Time lidar_handler_ros_time_frame_ts;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;

    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;

    std::vector<LidarScanProcessor> lidar_scan_handlers;

    LidarPacketAccumlator lidar_packet_accumlator;
};

}  // namespace ouster_ros