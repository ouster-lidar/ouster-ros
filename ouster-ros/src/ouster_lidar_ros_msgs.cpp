/**
 * @file ouster_lidar_ros_msgs.cpp
 * @brief Populate LidarInfo / LidarScan messages from SDK types.
 */

#include "ouster_ros/ouster_lidar_ros_msgs.h"

#include <ouster/chanfield.h>
#include <ouster/field.h>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace ouster_ros {

namespace {


using ouster::sdk::core::ChanFieldType;
using ouster::sdk::core::FieldClass;
using ouster::sdk::core::FieldType;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::field_type_size;
using ouster::sdk::core::destagger;

using sensor_msgs::msg::PointField;
using lidar_msgs::msg::LidarChannel;
using lidar_msgs::msg::LidarInfo;


// Map an SDK ChanFieldType primitive to the matching sensor_msgs/PointField tag.
// Returns 0 for non-primitive / unsupported types (VOID, UINT64, INT64).
uint8_t chan_type_to_point_field(ChanFieldType t) {
    switch (t) {
        case ChanFieldType::INT8:    return PointField::INT8;
        case ChanFieldType::UINT8:   return PointField::UINT8;
        case ChanFieldType::INT16:   return PointField::INT16;
        case ChanFieldType::UINT16:  return PointField::UINT16;
        case ChanFieldType::INT32:   return PointField::INT32;
        case ChanFieldType::UINT32:  return PointField::UINT32;
        case ChanFieldType::FLOAT32: return PointField::FLOAT32;
        case ChanFieldType::FLOAT64: return PointField::FLOAT64;
        case ChanFieldType::CHAR:    return PointField::UINT8;
        // TODO[unaal]: the following types are not supported by sensor_msgs/PointField.
        // case ChanFieldType::VOID:    return PointField::VOID;
        // case ChanFieldType::UINT64:  return PointField::UINT64;
        // case ChanFieldType::INT64:   return PointField::INT64;
        default:                     return 0;
    }
}

// Number of primitive elements per pixel; 1 for scalar fields, N for (H,W,N) fields.
size_t elements_per_pixel(const FieldType& ft) {
    size_t n = 1;
    for (size_t d : ft.extra_dims) n *= d;
    return n ? n : 1;
}

// A PIXEL_FIELD whose element type maps to a PointField primitive.
// Extra per-pixel dimensions are fine — they are represented with LidarChannel.count > 1.
bool is_publishable_pixel_field(const FieldType& ft) {
    return ft.field_class == FieldClass::PIXEL_FIELD &&
           chan_type_to_point_field(ft.element_type) != 0;
}

std::string to_lower_ascii(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
}

// TODO[unaal]: We probably want to use the per-pixel angles for DF and OS sensors instead of
// averaging. This would simplify the proposal and is more general.
// Average a (h*w) vector of degrees column-wise into an h-sized vector of radians.
// Used for DF-family sensors whose beam angles are per-pixel rather than per-row.
template <typename V>
std::vector<float> row_average_deg_to_rad(const V& src, size_t h, size_t w, bool negate) {
    std::vector<float> out(h);
    const double sign = negate ? -1.0 : 1.0;
    for (size_t r = 0; r < h; ++r) {
        double sum = 0.0;
        const size_t row_off = r * w;
        for (size_t c = 0; c < w; ++c) sum += src[row_off + c];
        out[r] = static_cast<float>(sign * (sum / static_cast<double>(w)) * M_PI / 180.0);
    }
    return out;
}

}  // namespace

void sensor_info_to_lidar_info(const SensorInfo& info, const std::string& frame_id,
                               LidarInfo& out) {
    out = LidarInfo{};
    out.header.frame_id = frame_id;

    // TODO[unaal]: revist this as we agreed to publish range data directly in SI metres.
    // Ouster RANGE is published as raw uint32 millimeters; scale to SI metres.
    out.range_multiplier = 0.001;
    out.range_offset = 0.0;

    const size_t w = info.format.columns_per_frame;
    const size_t h = info.format.pixels_per_column;

    // Azimuth sweeps CW by column (matches Ouster's default scan indexing); the
    // beam_azimuth_angles table carries the CW offset so downstream cos/sin can be
    // applied directly to (horizontal_angles[c] + beam_azimuth_angles[r]).
    out.horizontal_fov_min = 0.F;
    out.horizontal_fov_max = static_cast<float>(2.0 * M_PI);
    out.horizontal_angles.resize(w);
    const double azi_step = 2.0 * M_PI / static_cast<double>(w);
    for (size_t c = 0; c < w; ++c) {
        out.horizontal_angles[c] =
            static_cast<float>(2.0 * M_PI - static_cast<double>(c) * azi_step);
    }

    // Beam geometry: OS sensors use per-row tables (size == h), DF sensors use
    // per-pixel tables (size == h*w) which we row-average. Anything else leaves
    // the tables empty so LidarScanToPointCloud falls back to FOV interpolation.
    const size_t n_alt = info.beam_altitude_angles.size();
    const size_t n_azi = info.beam_azimuth_angles.size();
    if (n_alt == h && n_azi == h) {
        out.vertical_angles.resize(h);
        out.beam_azimuth_angles.resize(h);
        for (size_t r = 0; r < h; ++r) {
            out.vertical_angles[r] =
                static_cast<float>(info.beam_altitude_angles[r] * M_PI / 180.0);
            out.beam_azimuth_angles[r] =
                static_cast<float>(-info.beam_azimuth_angles[r] * M_PI / 180.0);
        }
    } else if (n_alt == h * w && n_azi == h * w) {
        out.vertical_angles = row_average_deg_to_rad(info.beam_altitude_angles, h, w, false);
        out.beam_azimuth_angles = row_average_deg_to_rad(info.beam_azimuth_angles, h, w, true);
    }

    // Keep fov_min/max consistent with the explicit table so consumers that only
    // look at the FOV fields don't observe a degenerate zero-width range.
    if (!out.vertical_angles.empty()) {
        const auto mm = std::minmax_element(out.vertical_angles.begin(),
                                            out.vertical_angles.end());
        out.vertical_fov_min = *mm.first;
        out.vertical_fov_max = *mm.second;
    } else {
        out.vertical_fov_min = 0.F;
        out.vertical_fov_max = 0.F;
    }
}

void ouster_sdk_lidar_scan_to_ros_msg(const ouster::sdk::core::LidarScan& ls,
                                      const rclcpp::Time& stamp, const std::string& frame_id,
                                      lidar_msgs::msg::LidarScan& out) {
    out = lidar_msgs::msg::LidarScan{};
    out.header.stamp = stamp;
    out.header.frame_id = frame_id;
    out.height = static_cast<uint32_t>(ls.h);
    out.width = static_cast<uint32_t>(ls.w);
    out.is_bigendian = 0;  // SDK uses host-endian; ROS2 always runs on little-endian platforms.

    // Select the PIXEL_FIELDs we can encode and freeze their order (sorted by name
    // so the wire layout is reproducible and diffable across runs).
    std::vector<FieldType> pixel_types;
    for (const auto& ft : ls.field_types()) {
        if (is_publishable_pixel_field(ft)) pixel_types.push_back(ft);
    }

    std::sort(pixel_types.begin(), pixel_types.end(),
              [](const FieldType& a, const FieldType& b) { return a.name < b.name; });

    // destagger the LidarScan before publishing (only copy fields that are publishable)
    // TODO[unaal]: we could optimize this by re-writing the ScanBatcher to produce destaggered LidarScan directly.
    ouster::sdk::core::LidarScan destaggered_ls(ls);
    // for (const auto& ft : ls.field_types()) {
    //     if (is_publishable_pixel_field(ft)) {
    //         destaggered_ls.field(ft.name) =
    //             destagger(*ls.sensor_info, ls.field(ft.name));
    //     }
    // }

    // Pass 1: assign each channel a contiguous byte range in data[]. Channels are
    // packed back-to-back in name order; no interleaving.
    out.channels.resize(pixel_types.size());
    const size_t num_pixels =
        static_cast<size_t>(out.height) * static_cast<size_t>(out.width);
    uint64_t total_bytes = 0;
    for (size_t i = 0; i < pixel_types.size(); ++i) {
        const auto& ft = pixel_types[i];
        const uint32_t count = static_cast<uint32_t>(elements_per_pixel(ft));
        const uint32_t es = static_cast<uint32_t>(field_type_size(ft.element_type));
        LidarChannel& ch = out.channels[i];
        ch.name = to_lower_ascii(ft.name);
        ch.offset = static_cast<uint32_t>(total_bytes);
        ch.datatype = chan_type_to_point_field(ft.element_type);
        ch.count = count;
        total_bytes += static_cast<uint64_t>(num_pixels) *
                       static_cast<uint64_t>(es) * static_cast<uint64_t>(count);
    }

    out.data.resize(static_cast<size_t>(total_bytes));
    if (total_bytes == 0) return;

    // Pass 2: one bulk memcpy per channel. The SDK already stores each PIXEL_FIELD
    // as a contiguous row-major (h * w * count) block, which is exactly the planar
    // layout the message specifies — so the copy is byte-for-byte identical to the
    // source buffer with no strides or reshuffling.
    for (size_t i = 0; i < pixel_types.size(); ++i) {
        const auto& ft = pixel_types[i];
        const auto& field = destaggered_ls.field(ft.name);
        // TODO[unaal]: probably a warning here is more preferable than an exception.
        if (field.sparse()) {
            throw std::runtime_error(
                "ouster_sdk_lidar_scan_to_ros_msg: sparse pixel field is not supported: " +
                ft.name);
        }
        const auto* src = reinterpret_cast<const uint8_t*>(field.get<void>());
        const size_t bytes_per_pixel =
            field_type_size(ft.element_type) * out.channels[i].count;
        std::memcpy(out.data.data() + out.channels[i].offset, src,
                    num_pixels * bytes_per_pixel);
    }
}

}  // namespace ouster_ros
