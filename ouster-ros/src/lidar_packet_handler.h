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

#include "lock_free_ring_buffer.h"
#include <thread>
#include <chrono>

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

}  // namespace

namespace ouster_ros {

namespace sensor = ouster::sensor;

using LidarScanProcessor =
    std::function<void(const ouster::LidarScan&, uint64_t, const rclcpp::Time&)>;

class LidarPacketHandler {
    using LidarPacketAccumlator =
        std::function<bool(const sensor::LidarPacket&)>;

   public:
    using HandlerOutput = ouster::LidarScan;

    using HandlerType = std::function<void(const sensor::LidarPacket&)>;

   public:
    LidarPacketHandler(const sensor::sensor_info& info,
                       const std::vector<LidarScanProcessor>& handlers,
                       const std::string& timestamp_mode,
                       int64_t ptp_utc_tai_offset,
                       float min_scan_valid_columns_ratio)
        : ring_buffer(LIDAR_SCAN_COUNT),
          lidar_scan_handlers{handlers},
          ptp_utc_tai_offset_(ptp_utc_tai_offset),
          min_scan_valid_columns_ratio_(min_scan_valid_columns_ratio) {
        // initialize lidar_scan processor and buffer
        scan_batcher = std::make_unique<ouster::ScanBatcher>(info);

        lidar_scans.resize(LIDAR_SCAN_COUNT);
        mutexes.resize(LIDAR_SCAN_COUNT);

        for (size_t i = 0; i < lidar_scans.size(); ++i) {
            lidar_scans[i] = std::make_unique<ouster::LidarScan>(
                info.format.columns_per_frame, info.format.pixels_per_column,
                info.format.udp_profile_lidar);
            mutexes[i] = std::make_unique<std::mutex>();
        }

        lidar_scans_processing_thread = std::make_unique<std::thread>([this]() {
            while (lidar_scans_processing_active) {
                process_scans();
            }
            RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                         "lidar_scans_processing_thread done.");
        });

        // initalize time handlers
        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.mode);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };

        const sensor::packet_format& pf = sensor::get_format(info);

        std::function<bool(LidarPacketHandler&, const sensor::packet_format&,
                           const sensor::LidarPacket&, ouster::LidarScan&)>
            lidar_handler;

        if (timestamp_mode == "TIME_FROM_ROS_TIME") {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_ros_time);
        } else if (timestamp_mode == "TIME_FROM_PTP_1588") {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_sensor_time_ptp);
        } else /*SENSOR TIME (INTERNAL_OSC, SYNC_PULSE_IN)*/ {
            lidar_handler =
                std::mem_fn(&LidarPacketHandler::lidar_handler_sensor_time);
        }

        lidar_packet_accumlator = LidarPacketAccumlator{
            [this, pf, lidar_handler](const sensor::LidarPacket& lidar_packet) {
                if (ring_buffer.full()) {
                    RCLCPP_WARN(rclcpp::get_logger(getName()),
                                "lidar_scans full, DROPPING PACKET");
                    return false;
                }
                bool result = false;
                {
                    std::unique_lock<std::mutex> lock(
                        *(mutexes[ring_buffer.write_head()]));
                    auto& lidar_scan = *lidar_scans[ring_buffer.write_head()];
                    result = lidar_handler(*this, pf, lidar_packet, lidar_scan);
                    if (result) {
                        // count the number of valid columns in the scan
                        auto status = lidar_scan.status();
                        size_t valid_cols = std::count_if(status.data(), status.data() + status.size(),
                               [](const uint32_t s) { return (s & 0x01); });
                        if (valid_cols < static_cast<size_t>(min_scan_valid_columns_ratio_ * status.size())) {
                            RCLCPP_WARN_STREAM(rclcpp::get_logger(getName()), "number of valid columns per scan "
                                << valid_cols << "/" << status.size()
                                <<" which is below the ratio " << std::setprecision(4) << (100 * min_scan_valid_columns_ratio_)
                                << "%, SKIPPING SCAN");
                            result = false;
                        }
                    }
                }
                if (result) {
                    ring_buffer.write();
                }
                return result;
            }};
    }

    LidarPacketHandler(const LidarPacketHandler&) = delete;
    LidarPacketHandler& operator=(const LidarPacketHandler&) = delete;
    ~LidarPacketHandler() {
        RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                     "LidarPacketHandler::~LidarPacketHandler()");
        if (lidar_scans_processing_thread->joinable()) {
            lidar_scans_processing_active = false;
            lidar_scans_processing_thread->join();
        }
    }

    void register_lidar_scan_handler(LidarScanProcessor handler) {
        lidar_scan_handlers.push_back(handler);
    }

    void clear_registered_lidar_scan_handlers() { lidar_scan_handlers.clear(); }

   public:
    static HandlerType create(
        const sensor::sensor_info& info,
        const std::vector<LidarScanProcessor>& handlers,
        const std::string& timestamp_mode, int64_t ptp_utc_tai_offset,
        float min_scan_valid_columns_ratio) {
        auto handler = std::make_shared<LidarPacketHandler>(
            info, handlers, timestamp_mode, ptp_utc_tai_offset,
            min_scan_valid_columns_ratio);
        return [handler](const sensor::LidarPacket& lidar_packet) {
            if (handler->lidar_packet_accumlator(lidar_packet)) {
                handler->ring_buffer_has_elements.notify_one();
            }
        };
    }

    const std::string getName() const { return "lidar_packet_hander"; }

    void process_scans() {
        {
            using namespace std::chrono;
            std::unique_lock<std::mutex> index_lock(ring_buffer_mutex);
            ring_buffer_has_elements.wait_for(
                index_lock, 1s, [this] { return !ring_buffer.empty(); });

            if (ring_buffer.empty()) return;
        }

        std::unique_lock<std::mutex> lock(*mutexes[ring_buffer.read_head()]);

        for (auto h : lidar_scan_handlers) {
            h(*lidar_scans[ring_buffer.read_head()], lidar_scan_estimated_ts,
              lidar_scan_estimated_msg_ts);
        }

        // when we hit percent amount of the ring_buffer capacity throttle
        size_t read_step = 1;
        if (ring_buffer.size() > THROTTLE_PERCENT * ring_buffer.capacity()) {
            RCLCPP_WARN(rclcpp::get_logger(getName()),
                               "lidar_scans %d%% full, THROTTLING",
                               static_cast<int>(100* THROTTLE_PERCENT));
            read_step = 2;
        }
        ring_buffer.read(read_step);
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
        return impl::ulround(interpolated_value);
    }

    uint64_t extrapolate_value(int curr_scan_first_nonzero_idx,
                               uint64_t curr_scan_first_nonzero_value) {
        double extrapolated_value =
            curr_scan_first_nonzero_value -
            scan_col_ts_spacing_ns * curr_scan_first_nonzero_idx;
        return impl::ulround(extrapolated_value);
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

    rclcpp::Time extrapolate_frame_ts(const sensor::packet_format& pf,
                                      const uint8_t* lidar_buf,
                                      const rclcpp::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(pf, lidar_buf);
        auto delta_time = rclcpp::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    bool lidar_handler_sensor_time(const sensor::packet_format&,
                                   const sensor::LidarPacket& lidar_packet,
                                   ouster::LidarScan& lidar_scan) {
        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        lidar_scan_estimated_ts = compute_scan_ts(lidar_scan.timestamp());
        lidar_scan_estimated_msg_ts = rclcpp::Time(lidar_scan_estimated_ts);

        return true;
    }

    bool lidar_handler_sensor_time_ptp(const sensor::packet_format&,
                                       const sensor::LidarPacket& lidar_packet,
                                       ouster::LidarScan& lidar_scan) {
        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        auto ts_v = lidar_scan.timestamp();
        for (int i = 0; i < ts_v.rows(); ++i)
            ts_v[i] = impl::ts_safe_offset_add(ts_v[i], ptp_utc_tai_offset_);
        lidar_scan_estimated_ts = compute_scan_ts(ts_v);
        lidar_scan_estimated_msg_ts =
            rclcpp::Time(lidar_scan_estimated_ts);

        return true;
    }

    bool lidar_handler_ros_time(const sensor::packet_format& pf,
                                const sensor::LidarPacket& lidar_packet,
                                ouster::LidarScan& lidar_scan) {
        auto packet_receive_time = rclcpp::Time(lidar_packet.host_timestamp);

        if (!lidar_handler_ros_time_frame_ts) {
            lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
                pf, lidar_packet.buf.data(),
                packet_receive_time);  // first point cloud time
        }

        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        lidar_scan_estimated_ts = compute_scan_ts(lidar_scan.timestamp());
        lidar_scan_estimated_msg_ts = lidar_handler_ros_time_frame_ts.value();
        // set time for next point cloud msg
        lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
            pf, lidar_packet.buf.data(), packet_receive_time);
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
    const int LIDAR_SCAN_COUNT = 10;
    const float THROTTLE_PERCENT = 0.7f;
    LockFreeRingBuffer ring_buffer;
    std::mutex ring_buffer_mutex;
    std::vector<std::unique_ptr<ouster::LidarScan>> lidar_scans;
    std::vector<std::unique_ptr<std::mutex>> mutexes;

    uint64_t lidar_scan_estimated_ts;
    rclcpp::Time lidar_scan_estimated_msg_ts;

    std::optional<rclcpp::Time> lidar_handler_ros_time_frame_ts;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;

    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    std::function<uint64_t(const ouster::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;

    std::vector<LidarScanProcessor> lidar_scan_handlers;

    LidarPacketAccumlator lidar_packet_accumlator;

    bool lidar_scans_processing_active = true;
    std::unique_ptr<std::thread> lidar_scans_processing_thread;
    std::condition_variable ring_buffer_has_elements;

    int64_t ptp_utc_tai_offset_;

    float min_scan_valid_columns_ratio_ = 0.0f;
};

}  // namespace ouster_ros