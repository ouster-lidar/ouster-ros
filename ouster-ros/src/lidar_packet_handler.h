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
#include <rclcpp/rclcpp.hpp>

#include "lock_free_ring_buffer.h"
#include <optional>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <vector>
#include <string>

#include <ouster/image_processing.h>

namespace ChanField = ouster::sdk::core::ChanField;

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

// Fast float16 -> float32 conversion for normal-range values.
inline float f16_bits_to_f32(uint16_t bits) {
    if (bits == 0) return 0.0f;
    const uint32_t expanded = static_cast<uint32_t>(bits + 0x1C000u) << 13;
    float result;
    std::memcpy(&result, &expanded, sizeof(float));
    return result;
}

inline uint8_t f32_to_u8(float v) {
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    return static_cast<uint8_t>(v * 255.0f + 0.5f);
}

}  // namespace

namespace ouster_ros {

using LidarScanProcessor =
    std::function<void(const ouster::sdk::core::LidarScan&, uint64_t, const rclcpp::Time&)>;

class LidarPacketHandler {
    using LidarPacketAccumlator =
        std::function<bool(const ouster::sdk::core::LidarPacket&)>;

   public:
    using HandlerOutput = ouster::sdk::core::LidarScan;

    using HandlerType = std::function<void(const ouster::sdk::core::LidarPacket&)>;

   public:
    LidarPacketHandler(const ouster::sdk::core::SensorInfo& info,
                       const std::vector<LidarScanProcessor>& handlers,
                       const std::string& timestamp_mode,
                       int64_t ptp_utc_tai_offset,
                       float min_scan_valid_columns_ratio)
        : ring_buffer(LIDAR_SCAN_COUNT),
          lidar_scan_handlers{handlers},
          ptp_utc_tai_offset_(ptp_utc_tai_offset),
          min_scan_valid_columns_ratio_(min_scan_valid_columns_ratio) {
        // initialize lidar_scan processor and buffer
        scan_batcher = std::make_unique<ouster::sdk::core::ScanBatcher>(info);

        lidar_scans.resize(LIDAR_SCAN_COUNT);
        mutexes.resize(LIDAR_SCAN_COUNT);
        // Per-slot scan-timestamp storage. Kept alongside lidar_scans so the
        // existing per-slot mutex serialises writer/reader access.
        lidar_scan_slot_ts.assign(LIDAR_SCAN_COUNT, 0);
        lidar_scan_slot_msg_ts.assign(LIDAR_SCAN_COUNT, rclcpp::Time(0, 0));

        for (size_t i = 0; i < lidar_scans.size(); ++i) {
            // NOTE: must construct with SensorInfo (not the
            // (w, h, UDPProfileLidar, cols_per_packet) overload) so that the
            // RGB profile's RGB channel is created as a 3D (h x w x 3) field.
            lidar_scans[i] = std::make_unique<ouster::sdk::core::LidarScan>(info);
            mutexes[i] = std::make_unique<std::mutex>();
        }

        if (info.format.udp_profile_lidar == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16 ||
            info.format.udp_profile_lidar == ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL) {
            has_rgb_ = true;
            uint32_t H = info.format.pixels_per_column;
            uint32_t W = info.format.columns_per_frame;
            r_field_float = ouster::sdk::core::img_t<float>(H, W);
            g_field_float = ouster::sdk::core::img_t<float>(H, W);
            b_field_float = ouster::sdk::core::img_t<float>(H, W);
            auto_exposure_ = std::make_unique<ouster::sdk::core::image::AutoExposure>();
            for (auto& ls : lidar_scans) {
                using ouster::sdk::core::fd_array;
                ls->add_field(ChanField::R_U8, fd_array<uint8_t>(H, W));
                ls->add_field(ChanField::G_U8, fd_array<uint8_t>(H, W));
                ls->add_field(ChanField::B_U8, fd_array<uint8_t>(H, W));
            }
        }

        lidar_scans_processing_thread = std::make_unique<std::thread>([this]() {
            while (lidar_scans_processing_active) {
                process_scans();
            }
            RCLCPP_DEBUG(rclcpp::get_logger(getName()),
                         "lidar_scans_processing_thread done.");
        });

        // initalize time handlers
        scan_col_ts_spacing_ns = compute_scan_col_ts_spacing_ns(info.config.lidar_mode.value());
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };

        const ouster::sdk::core::PacketFormat& pf = ouster::sdk::core::get_format(info);

        std::function<bool(LidarPacketHandler&, const ouster::sdk::core::PacketFormat&,
                           const ouster::sdk::core::LidarPacket&, ouster::sdk::core::LidarScan&)>
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
            [this, pf, lidar_handler](const ouster::sdk::core::LidarPacket& lidar_packet) {
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
        const ouster::sdk::core::SensorInfo& info,
        const std::vector<LidarScanProcessor>& handlers,
        const std::string& timestamp_mode, int64_t ptp_utc_tai_offset,
        float min_scan_valid_columns_ratio) {
        auto handler = std::make_shared<LidarPacketHandler>(
            info, handlers, timestamp_mode, ptp_utc_tai_offset,
            min_scan_valid_columns_ratio);
        return [handler](const ouster::sdk::core::LidarPacket& lidar_packet) {
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

        const auto slot = ring_buffer.read_head();
        std::unique_lock<std::mutex> lock(*mutexes[slot]);

        // apply auto exposure to the rgb data only if the point cloud has rgb fields
        // NOTE[UN]: Don't copy the lidar scan to avoid modifying the original scan in the ring buffer
        ouster::sdk::core::LidarScan& ls = *lidar_scans[slot];

        if (has_rgb_) {
            using ouster::sdk::core::img_t;
            Eigen::Ref<img_t<uint16_t>> r_field = ls.field<uint16_t>(ChanField::R);
            Eigen::Ref<img_t<uint16_t>> g_field = ls.field<uint16_t>(ChanField::G);
            Eigen::Ref<img_t<uint16_t>> b_field = ls.field<uint16_t>(ChanField::B);

            // Get raw pointers for speed
            const uint16_t* r_in = r_field.data();
            const uint16_t* g_in = g_field.data();
            const uint16_t* b_in = b_field.data();

            float* r_out = r_field_float.data();
            float* g_out = g_field_float.data();
            float* b_out = b_field_float.data();

            for (Eigen::Index i = 0; i < r_field.size(); ++i) {
                r_out[i] = f16_bits_to_f32(r_in[i]);
                g_out[i] = f16_bits_to_f32(g_in[i]);
                b_out[i] = f16_bits_to_f32(b_in[i]);
            }

            auto_exposure_->update(r_field_float, g_field_float, b_field_float, true);

            Eigen::Ref<img_t<uint8_t>> r_field_uint8 = ls.field<uint8_t>(ChanField::R_U8);
            Eigen::Ref<img_t<uint8_t>> g_field_uint8 = ls.field<uint8_t>(ChanField::G_U8);
            Eigen::Ref<img_t<uint8_t>> b_field_uint8 = ls.field<uint8_t>(ChanField::B_U8);

            uint8_t* r_out_uint8 = r_field_uint8.data();
            uint8_t* g_out_uint8 = g_field_uint8.data();
            uint8_t* b_out_uint8 = b_field_uint8.data();

            for (Eigen::Index i = 0; i < r_field_uint8.size(); ++i) {
                r_out_uint8[i] = f32_to_u8(r_out[i]);
                g_out_uint8[i] = f32_to_u8(g_out[i]);
                b_out_uint8[i] = f32_to_u8(b_out[i]);
            }
        }

        for (auto h : lidar_scan_handlers) {
            h(ls, lidar_scan_slot_ts[slot], lidar_scan_slot_msg_ts[slot]);
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
        if (scan_width + curr_scan_first_nonzero_idx <=
            last_scan_last_nonzero_idx) {
            // Precondition violated (corrupt column indices); the assert above
            // is compiled out under NDEBUG and linear_interpolate would divide
            // by zero. Fall back to extrapolating from the current scan alone.
            return extrapolate_value(curr_scan_first_nonzero_idx,
                                     curr_scan_first_nonzero_value);
        }
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

    // compute_scan_ts_0 for first scan. Returns std::nullopt if the scan has
    // no non-zero timestamp column at all (the caller must drop the scan);
    // the asserts this replaces were compiled out under NDEBUG and an
    // all-zero column would dereference an end iterator.
    std::optional<uint64_t> compute_scan_ts_0(
        const ouster::sdk::core::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) {
            RCLCPP_WARN(rclcpp::get_logger(getName()),
                        "scan timestamp column is all zeros, dropping scan");
            return std::nullopt;
        }
        int curr_scan_first_nonzero_idx = idx - ts_v.data();
        uint64_t curr_scan_first_nonzero_value = *idx;

        uint64_t scan_ns =
            curr_scan_first_nonzero_idx == 0
                ? curr_scan_first_nonzero_value
                : extrapolate_value(curr_scan_first_nonzero_idx,
                                    curr_scan_first_nonzero_value);

        last_scan_last_nonzero_idx =
            find_if_reverse(ts_v, [](uint64_t h) { return h != 0; });
        // Logically guaranteed to succeed since std::find_if above already
        // located a non-zero element, but guard in Release: indexing ts_v
        // with -1 would be UB. State is untouched, so the next scan
        // re-anchors through compute_scan_ts_0 again.
        if (last_scan_last_nonzero_idx < 0) return std::nullopt;
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_n(ts_v);
        };

        return scan_ns;
    }

    // compute_scan_ts_n applied to all subsequent scans except first one.
    // Same std::nullopt contract as compute_scan_ts_0; when a scan is
    // dropped the estimator re-anchors on the next valid scan instead of
    // imputing across the gap with stale last_scan_last_nonzero_* state.
    std::optional<uint64_t> compute_scan_ts_n(
        const ouster::sdk::core::LidarScan::Header<uint64_t>& ts_v) {
        auto idx = std::find_if(ts_v.data(), ts_v.data() + ts_v.size(),
                                [](uint64_t h) { return h != 0; });
        if (idx == ts_v.data() + ts_v.size()) {
            RCLCPP_WARN(rclcpp::get_logger(getName()),
                        "scan timestamp column is all zeros, dropping scan");
            reset_compute_scan_ts();
            return std::nullopt;
        }
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
        // See compute_scan_ts_0: cannot fail after find_if succeeded, but
        // guard in Release and re-anchor rather than keep stale state.
        if (last_scan_last_nonzero_idx < 0) {
            reset_compute_scan_ts();
            return std::nullopt;
        }
        last_scan_last_nonzero_value = ts_v(last_scan_last_nonzero_idx);
        return scan_ns;
    }

    // Forget the previous scan's trailing timestamp and anchor the estimator
    // on the next valid scan via compute_scan_ts_0.
    void reset_compute_scan_ts() {
        last_scan_last_nonzero_idx = -1;
        last_scan_last_nonzero_value = 0;
        compute_scan_ts = [this](const auto& ts_v) {
            return compute_scan_ts_0(ts_v);
        };
    }

    uint16_t packet_col_index(const ouster::sdk::core::PacketFormat& pf,
                              const uint8_t* lidar_buf) {
        return pf.col_measurement_id(pf.nth_col(0, lidar_buf));
    }

    rclcpp::Time extrapolate_frame_ts(const ouster::sdk::core::PacketFormat& pf,
                                      const uint8_t* lidar_buf,
                                      const rclcpp::Time current_time) {
        auto curr_scan_first_arrived_idx = packet_col_index(pf, lidar_buf);
        auto delta_time = rclcpp::Duration(
            0,
            std::lround(scan_col_ts_spacing_ns * curr_scan_first_arrived_idx));
        return current_time - delta_time;
    }

    bool lidar_handler_sensor_time(const ouster::sdk::core::PacketFormat&,
                                   const ouster::sdk::core::LidarPacket& lidar_packet,
                                   ouster::sdk::core::LidarScan& lidar_scan) {
        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        const auto scan_ts = compute_scan_ts(lidar_scan.timestamp());
        if (!scan_ts) return false;  // drop scans without a usable timestamp
        const auto slot = ring_buffer.write_head();
        lidar_scan_slot_ts[slot] = *scan_ts;
        lidar_scan_slot_msg_ts[slot] = rclcpp::Time(*scan_ts);

        return true;
    }

    bool lidar_handler_sensor_time_ptp(const ouster::sdk::core::PacketFormat&,
                                       const ouster::sdk::core::LidarPacket& lidar_packet,
                                       ouster::sdk::core::LidarScan& lidar_scan) {
        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        auto ts_v = lidar_scan.timestamp();
        for (int i = 0; i < ts_v.rows(); ++i)
            ts_v[i] = impl::ts_safe_offset_add(ts_v[i], ptp_utc_tai_offset_);
        const auto scan_ts = compute_scan_ts(ts_v);
        if (!scan_ts) return false;  // drop scans without a usable timestamp
        const auto slot = ring_buffer.write_head();
        lidar_scan_slot_ts[slot] = *scan_ts;
        lidar_scan_slot_msg_ts[slot] = rclcpp::Time(*scan_ts);

        return true;
    }

    bool lidar_handler_ros_time(const ouster::sdk::core::PacketFormat& pf,
                                const ouster::sdk::core::LidarPacket& lidar_packet,
                                ouster::sdk::core::LidarScan& lidar_scan) {
        auto packet_receive_time = rclcpp::Time(lidar_packet.host_timestamp);

        if (!lidar_handler_ros_time_frame_ts) {
            lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
                pf, lidar_packet.buf.data(),
                packet_receive_time);  // first point cloud time
        }

        if (!(*scan_batcher)(lidar_packet, lidar_scan)) return false;
        const auto scan_ts = compute_scan_ts(lidar_scan.timestamp());
        const auto frame_ts = lidar_handler_ros_time_frame_ts.value();
        // set time for next point cloud msg (even when this scan is dropped,
        // so the next frame's ROS time stays anchored to packet arrival)
        lidar_handler_ros_time_frame_ts = extrapolate_frame_ts(
            pf, lidar_packet.buf.data(), packet_receive_time);
        if (!scan_ts) return false;  // drop scans without a usable timestamp
        const auto slot = ring_buffer.write_head();
        lidar_scan_slot_ts[slot] = *scan_ts;
        lidar_scan_slot_msg_ts[slot] = frame_ts;
        return true;
    }

    static double compute_scan_col_ts_spacing_ns(ouster::sdk::core::LidarMode ld_mode) {
        const auto scan_width = ouster::sdk::core::n_cols_of_lidar_mode(ld_mode);
        const auto scan_frequency = ouster::sdk::core::frequency_of_lidar_mode(ld_mode);
        const double one_sec_in_ns = 1e+9;
        return one_sec_in_ns / (scan_width * scan_frequency);
    }

   private:
    std::unique_ptr<ouster::sdk::core::ScanBatcher> scan_batcher;
    const int LIDAR_SCAN_COUNT = 10;
    const float THROTTLE_PERCENT = 0.7f;
    LockFreeRingBuffer ring_buffer;
    std::mutex ring_buffer_mutex;
    std::vector<std::unique_ptr<ouster::sdk::core::LidarScan>> lidar_scans;
    std::vector<std::unique_ptr<std::mutex>> mutexes;
    // Per-slot scan timestamp, written by the lidar_handler_* methods inside
    // the writer's per-slot mutex and read in process_scans() inside the same
    // mutex for the slot at read_head().
    std::vector<uint64_t> lidar_scan_slot_ts;
    std::vector<rclcpp::Time> lidar_scan_slot_msg_ts;

    std::optional<rclcpp::Time> lidar_handler_ros_time_frame_ts;

    int last_scan_last_nonzero_idx = -1;
    uint64_t last_scan_last_nonzero_value = 0;

    double scan_col_ts_spacing_ns;  // interval or spacing between columns of a
                                    // scan

    // Returns std::nullopt when the scan carries no usable timestamp (the
    // scan must then be dropped instead of published with a zero stamp).
    std::function<std::optional<uint64_t>(
        const ouster::sdk::core::LidarScan::Header<uint64_t>&)>
        compute_scan_ts;

    std::vector<LidarScanProcessor> lidar_scan_handlers;

    LidarPacketAccumlator lidar_packet_accumlator;

    std::atomic<bool> lidar_scans_processing_active = true;
    std::unique_ptr<std::thread> lidar_scans_processing_thread;
    std::condition_variable ring_buffer_has_elements;

    int64_t ptp_utc_tai_offset_;

    float min_scan_valid_columns_ratio_ = 0.0f;

    bool has_rgb_ = false;
    ouster::sdk::core::img_t<float> r_field_float;
    ouster::sdk::core::img_t<float> g_field_float;
    ouster::sdk::core::img_t<float> b_field_float;

    std::unique_ptr<ouster::sdk::core::image::AutoExposure> auto_exposure_;
};

}  // namespace ouster_ros