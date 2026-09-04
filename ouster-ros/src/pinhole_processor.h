// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file pinhole_processor.h
 * @brief Resamples destaggered lidar images into calibrated pinhole panels.
 *
 * range_image retains os_image's radial-range encoding: uint16 with 4 mm per
 * increment. depth_image is calibrated optical-axis depth in metres (32FC1)
 * for standard ROS depth consumers. When requested, unsupported outer pixels
 * are physically cropped and CameraInfo.roi records that crop in full-panel
 * coordinates.
 */

#pragma once

// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/image_processing.h"
#include "ouster/xyzlut.h"
#include "lidar_packet_handler.h"

namespace ouster_ros {

namespace ChanField = ouster::sdk::core::ChanField;

class PinholeProcessor {
   public:
    // Bound persistent image and lookup-table allocations from parameters.
    static constexpr uint32_t MAX_PANEL_DIMENSION = 8192;
    static constexpr uint64_t MAX_PANEL_PIXELS = 4U * 1024U * 1024U;
    static constexpr uint64_t MAX_TOTAL_PANEL_PIXELS = 8U * 1024U * 1024U;

    struct PanelConfig {
        std::string name;
        double yaw_rad = 0.0;       // CCW about lidar_frame +Z
        double pitch_rad = 0.0;     // up positive (about panel-yawed +Y)
        double hfov_rad = M_PI / 2.0;
        uint32_t width = 256;
        uint32_t height = 0;        // 0: auto-fit VFOV at square pixels
        double vfov_rad = 0.0;      // 0: derive/retain square pixels
        bool crop_to_valid_region = true;
    };

    using pixel_type = uint16_t;

    struct PanelOutput {
        std::string name;
        std::string optical_frame_id;
        double yaw_rad = 0.0;
        double pitch_rad = 0.0;
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>> images;
        // Metric optical-axis depth images keyed by RANGE / RANGE2. These are
        // separate from images because monochrome display outputs are uint16,
        // optional RGB is RGB8, and depth is 32FC1 in metres.
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>>
            depth_images;
        sensor_msgs::msg::CameraInfo camera_info;
        // LUT in source space. Its dimensions match the emitted (possibly
        // cropped) images, while CameraInfo.roi maps its pixels back into the
        // configured full-panel coordinates:
        //   r_src(u, v) = source row (-1 means unsupported)
        //   v_src(u, v) = source destaggered column
        //   raw_v_src(u, v) = source staggered/raw column
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> r_src;
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> v_src;
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            raw_v_src;
        // depth = raw_range * depth_scale + depth_offset, in metres.
        Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            depth_scale;
        Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            depth_offset;
    };

    using OutputType = std::vector<std::shared_ptr<PanelOutput>>;
    using PostProcessingFn = std::function<void(OutputType&)>;

    PinholeProcessor(const ouster::sdk::core::SensorInfo& info,
                     const std::vector<PanelConfig>& panel_configs,
                     const std::string& optical_frame_template,
                     const std::string& lidar_namespace,
                     double azimuth_offset_columns,
                     PostProcessingFn func, bool publish_rgb = false)
        : info_(info),
          azimuth_offset_rad_(0.0),
          post_processing_fn_(func),
          has_rgb_(publish_rgb && profile_has_rgb(info)) {
        if (panel_configs.empty()) {
            throw std::invalid_argument("at least one panel is required");
        }
        const auto xyz_lut = make_lidar_xyz_lut(info_);
        const double normalized_offset_columns = normalize_columns(
            azimuth_offset_columns, info_.format.columns_per_frame);
        azimuth_offset_rad_ =
            normalized_offset_columns * 2.0 * M_PI /
            static_cast<double>(info_.format.columns_per_frame);
        std::string ns = lidar_namespace;
        while (!ns.empty() && ns.front() == '/') ns.erase(0, 1);

        uint64_t total_panel_pixels = 0;
        for (const auto& cfg : panel_configs) {
            auto panel = build_panel(cfg, ns, optical_frame_template, xyz_lut,
                                     total_panel_pixels);
            panels_.push_back(panel);
        }
    }

    const std::vector<std::shared_ptr<PanelOutput>>& panels() const {
        return panels_;
    }

    static std::string optical_frame_id(const std::string& frame_template,
                                        const std::string& lidar_namespace,
                                        const std::string& panel_name) {
        return substitute_template(frame_template, lidar_namespace,
                                   panel_name);
    }

    static std::map<std::string, std::string> channel_topics(
        const ouster::sdk::core::SensorInfo& info,
        bool publish_rgb = false) {
        return channel_topic_map_for(info.num_returns(),
                                     publish_rgb && profile_has_rgb(info));
    }

    static std::map<std::string, std::string> depth_topics(int n_returns) {
        return depth_topic_map_for(n_returns);
    }

    static LidarScanProcessor create(
        const ouster::sdk::core::SensorInfo& info,
        const std::vector<PanelConfig>& panel_configs,
        const std::string& optical_frame_template,
        const std::string& lidar_namespace,
        double azimuth_offset_columns,
        PostProcessingFn func, bool publish_rgb = false) {
        auto handler = std::make_shared<PinholeProcessor>(
            info, panel_configs, optical_frame_template, lidar_namespace,
            azimuth_offset_columns, func, publish_rgb);
        return [handler](const ouster::sdk::core::LidarScan& scan,
                         uint64_t scan_ts,
                         const rclcpp::Time& msg_ts) {
            handler->process(scan, scan_ts, msg_ts);
        };
    }

   private:
    std::shared_ptr<PanelOutput> build_panel(
        const PanelConfig& cfg, const std::string& ns,
        const std::string& optical_frame_template,
        const ouster::sdk::core::XYZLut& xyz_lut,
        uint64_t& total_panel_pixels) {
        auto out = std::make_shared<PanelOutput>();
        out->name = cfg.name;
        out->yaw_rad = cfg.yaw_rad;
        out->pitch_rad = cfg.pitch_rad;
        out->optical_frame_id = optical_frame_id(
            optical_frame_template, ns, cfg.name);

        if (cfg.name.empty()) {
            throw std::invalid_argument("panel name must not be empty");
        }
        if (out->optical_frame_id.empty()) {
            throw std::invalid_argument("panel optical frame must not be empty");
        }
        if (!std::isfinite(cfg.yaw_rad) || !std::isfinite(cfg.pitch_rad)) {
            throw std::invalid_argument("panel yaw and pitch must be finite");
        }

        const uint32_t pw = cfg.width;
        if (pw == 0 || pw > MAX_PANEL_DIMENSION) {
            throw std::invalid_argument("panel width out of range");
        }
        if (!std::isfinite(cfg.hfov_rad) || cfg.hfov_rad <= 0.0 ||
            cfg.hfov_rad >= M_PI) {
            throw std::invalid_argument("panel hfov out of range");
        }
        if (cfg.height > MAX_PANEL_DIMENSION) {
            throw std::invalid_argument("panel height out of range");
        }
        if (!std::isfinite(cfg.vfov_rad) || cfg.vfov_rad < 0.0 ||
            cfg.vfov_rad >= M_PI) {
            throw std::invalid_argument("panel vfov out of range");
        }

        const double half_hfov = 0.5 * cfg.hfov_rad;
        const double fx = static_cast<double>(pw) / (2.0 * std::tan(half_hfov));
        if (!std::isfinite(fx) || fx <= 0.0) {
            throw std::invalid_argument("panel focal length out of range");
        }
        double fy = fx;

        uint32_t ph = cfg.height;
        if (ph == 0) {
            const double vfov_rad = (cfg.vfov_rad > 0.0)
                ? cfg.vfov_rad
                : derive_lidar_vfov_rad(cfg.pitch_rad);
            double ph_d = std::round(2.0 * fy * std::tan(0.5 * vfov_rad));
            if (!std::isfinite(ph_d) || ph_d < 1.0) ph_d = 1.0;
            const uint64_t max_height = std::min<uint64_t>(
                MAX_PANEL_DIMENSION, MAX_PANEL_PIXELS / pw);
            if (ph_d > static_cast<double>(max_height)) {
                ph_d = static_cast<double>(max_height);
            }
            ph = static_cast<uint32_t>(ph_d);
        } else if (cfg.vfov_rad > 0.0) {
            // Supplying both dimensions and both fields of view defines a
            // fully independent vertical focal length. Leave vfov at zero to
            // request square pixels instead.
            fy = static_cast<double>(ph) /
                 (2.0 * std::tan(0.5 * cfg.vfov_rad));
            if (!std::isfinite(fy) || fy <= 0.0) {
                throw std::invalid_argument(
                    "panel vertical focal length out of range");
            }
        }

        const uint64_t panel_pixels = static_cast<uint64_t>(pw) * ph;
        if (panel_pixels == 0 || panel_pixels > MAX_PANEL_PIXELS) {
            throw std::invalid_argument("panel pixel count out of range");
        }
        if (total_panel_pixels > MAX_TOTAL_PANEL_PIXELS - panel_pixels) {
            throw std::invalid_argument("total panel pixel count out of range");
        }
        total_panel_pixels += panel_pixels;

        // OpenCV/ROS pixel coordinates address pixel centres at integer
        // coordinates. Put the principal point at the centre of that grid;
        // for an even dimension it therefore lies between the two middle
        // pixels. The configured FOV continues to span the outer pixel edges.
        const double cx = 0.5 * static_cast<double>(pw - 1);
        const double cy = 0.5 * static_cast<double>(ph - 1);

        out->camera_info.header.frame_id = out->optical_frame_id;
        out->camera_info.width = pw;
        out->camera_info.height = ph;
        out->camera_info.distortion_model = "plumb_bob";
        out->camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        out->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        out->camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        out->camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0,
                              0.0, 0.0, 1.0, 0.0};
        compute_lut(*out, xyz_lut, fx, fy, cx, cy, pw, ph);
        apply_valid_crop(*out, cfg.crop_to_valid_region, pw, ph);

        const uint32_t output_width = out->camera_info.roi.width;
        const uint32_t output_height = out->camera_info.roi.height;

        const auto channel_topic_map = channel_topic_map_for(
            info_.num_returns(), has_rgb_);
        for (const auto& kv : channel_topic_map) {
            auto img = std::make_shared<sensor_msgs::msg::Image>();
            img->header.frame_id = out->optical_frame_id;
            img->width = output_width;
            img->height = output_height;
            img->is_bigendian = 0;
            if (kv.first == ChanField::RGB) {
                img->encoding = sensor_msgs::image_encodings::RGB8;
                img->step = output_width * 3 * sizeof(uint8_t);
            } else {
                img->encoding = sensor_msgs::image_encodings::MONO16;
                img->step = output_width * sizeof(pixel_type);
            }
            img->data.assign(static_cast<size_t>(img->step) * output_height,
                             0);
            out->images[kv.first] = img;
        }

        const auto depth_topic_map = depth_topic_map_for(info_.num_returns());
        for (const auto& kv : depth_topic_map) {
            auto img = std::make_shared<sensor_msgs::msg::Image>();
            img->header.frame_id = out->optical_frame_id;
            img->width = output_width;
            img->height = output_height;
            img->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            img->is_bigendian = 0;
            img->step = output_width * sizeof(float);
            img->data.resize(static_cast<size_t>(img->step) * output_height);
            const float invalid_depth =
                std::numeric_limits<float>::quiet_NaN();
            for (size_t offset = 0; offset < img->data.size();
                 offset += sizeof(float)) {
                std::memcpy(img->data.data() + offset, &invalid_depth,
                            sizeof(float));
            }
            out->depth_images[kv.first] = img;
        }

        return out;
    }

    void compute_lut(PanelOutput& out,
                     const ouster::sdk::core::XYZLut& xyz_lut, double fx,
                     double fy, double cx, double cy, uint32_t pw,
                     uint32_t ph) const {
        out.r_src.resize(ph, pw);
        out.v_src.resize(ph, pw);
        out.raw_v_src.resize(ph, pw);
        out.depth_scale.resize(ph, pw);
        out.depth_offset.resize(ph, pw);

        const auto& alts = info_.beam_altitude_angles;  // degrees, top to bottom
        const uint32_t H = info_.format.pixels_per_column;
        const uint32_t W = info_.format.columns_per_frame;

        // azimuth_offset_rad_ says where the native SDK lidar +X axis points
        // in the advertised parent frame. Convert the configured parent-frame
        // panel yaw back into SDK lidar coordinates before selecting samples
        // or projecting their optical-axis depth.
        const double lidar_yaw = out.yaw_rad - azimuth_offset_rad_;
        const double cosy = std::cos(lidar_yaw);
        const double siny = std::sin(lidar_yaw);
        // Positive pitch tilts the panel's forward axis toward lidar +Z.
        const double cosp = std::cos(out.pitch_rad);
        const double sinp = std::sin(out.pitch_rad);
        const double forward_x = cosy * cosp;
        const double forward_y = siny * cosp;
        const double forward_z = sinp;

        // A nearest-neighbour sample owns the angular cell extending halfway
        // to its neighbour. Use the actual edge spacings rather than an
        // average across potentially nonuniform beam altitude angles.
        std::vector<double> sorted_alts(alts.begin(), alts.end());
        std::sort(sorted_alts.begin(), sorted_alts.end());
        const double min_alt = sorted_alts.front();
        const double max_alt = sorted_alts.back();
        double lower_half_spacing = 0.25;
        double upper_half_spacing = 0.25;
        for (size_t i = 1; i < sorted_alts.size(); ++i) {
            if (sorted_alts[i] > min_alt) {
                lower_half_spacing = 0.5 * (sorted_alts[i] - min_alt);
                break;
            }
        }
        for (size_t i = sorted_alts.size(); i > 1; --i) {
            if (sorted_alts[i - 2] < max_alt) {
                upper_half_spacing =
                    0.5 * (max_alt - sorted_alts[i - 2]);
                break;
            }
        }
        const double min_supported_elev = min_alt - lower_half_spacing;
        const double max_supported_elev = max_alt + upper_half_spacing;

        for (uint32_t u = 0; u < ph; ++u) {
            for (uint32_t v = 0; v < pw; ++v) {
                const double x_o = (static_cast<double>(v) - cx) / fx;
                const double y_o = (static_cast<double>(u) - cy) / fy;
                const double z_o = 1.0;

                // Optical child coordinates to panel body coordinates.
                const double pb_x = z_o;
                const double pb_y = -x_o;
                const double pb_z = -y_o;

                const double pp_x = cosp * pb_x - sinp * pb_z;
                const double pp_y = pb_y;
                const double pp_z = sinp * pb_x + cosp * pb_z;

                const double lx = cosy * pp_x - siny * pp_y;
                const double ly = siny * pp_x + cosy * pp_y;
                const double lz = pp_z;

                const double azimuth = std::atan2(ly, lx);
                const double horiz = std::sqrt(lx * lx + ly * ly);
                const double elev_rad = std::atan2(lz, horiz);
                const double elev_deg = elev_rad * 180.0 / M_PI;

                int32_t r_src = -1;
                double best_diff = std::numeric_limits<double>::infinity();
                for (size_t r = 0; r < alts.size(); ++r) {
                    const double diff = std::abs(alts[r] - elev_deg);
                    if (diff < best_diff) {
                        best_diff = diff;
                        r_src = static_cast<int32_t>(r);
                    }
                }
                if (elev_deg < min_supported_elev ||
                    elev_deg > max_supported_elev) {
                    r_src = -1;
                }

                // Select the raw measurement column from the calibrated beam
                // azimuth. pixel_shift_by_row is an image destaggering offset,
                // not a geometric substitute for beam_azimuth_angles; the two
                // differ substantially for some products and firmware.
                // make_xyz_lut defines a raw point's azimuth as:
                //   -raw_col * 2*pi/W - beam_azimuth[row].
                const double az_norm = normalize_radians(azimuth);
                int32_t valid_r =
                    (r_src >= 0 && r_src < static_cast<int32_t>(H))
                        ? r_src
                        : -1;

                int32_t raw_v = -1;
                int32_t v_src = 0;
                if (valid_r >= 0) {
                    const double beam_azimuth_rad =
                        info_.beam_azimuth_angles[valid_r] * M_PI / 180.0;
                    const double raw_column = normalize_columns(
                        -(az_norm + beam_azimuth_rad) *
                            static_cast<double>(W) / (2.0 * M_PI),
                        W);
                    // raw_column is in [0, W), but rounding can land on W.
                    raw_v = wrap_index(std::lround(raw_column),
                                       static_cast<int32_t>(W));
                    // The display channels are sampled after SDK-compatible
                    // destaggering, whose output column v reads raw v-shift.
                    v_src = raw_to_destaggered_column(
                        raw_v, info_.format.pixel_shift_by_row[valid_r],
                        static_cast<int32_t>(W));
                    if (!raw_column_in_window(raw_v)) {
                        valid_r = -1;
                        raw_v = -1;
                    }
                }
                out.v_src(u, v) = v_src;
                out.r_src(u, v) = valid_r;
                out.raw_v_src(u, v) = raw_v;

                if (valid_r < 0) {
                    out.depth_scale(u, v) =
                        std::numeric_limits<float>::quiet_NaN();
                    out.depth_offset(u, v) = 0.0f;
                    continue;
                }

                const Eigen::Index source_index =
                    static_cast<Eigen::Index>(valid_r) * W + raw_v;
                out.depth_scale(u, v) = static_cast<float>(
                    forward_x * xyz_lut.direction(source_index, 0) +
                    forward_y * xyz_lut.direction(source_index, 1) +
                    forward_z * xyz_lut.direction(source_index, 2));
                out.depth_offset(u, v) = static_cast<float>(
                    forward_x * xyz_lut.offset(source_index, 0) +
                    forward_y * xyz_lut.offset(source_index, 1) +
                    forward_z * xyz_lut.offset(source_index, 2));
            }
        }
    }

    void apply_valid_crop(PanelOutput& out, bool crop_to_valid_region,
                          uint32_t full_width, uint32_t full_height) const {
        uint32_t min_u = full_height;
        uint32_t min_v = full_width;
        uint32_t max_u = 0;
        uint32_t max_v = 0;
        bool found_valid_pixel = false;
        for (uint32_t u = 0; u < full_height; ++u) {
            for (uint32_t v = 0; v < full_width; ++v) {
                if (out.r_src(u, v) < 0) continue;
                found_valid_pixel = true;
                min_u = std::min(min_u, u);
                min_v = std::min(min_v, v);
                max_u = std::max(max_u, u);
                max_v = std::max(max_v, v);
            }
        }
        if (!found_valid_pixel && crop_to_valid_region) {
            throw std::invalid_argument(
                "panel '" + out.name +
                "' frustum does not overlap the configured lidar data");
        }

        if (!crop_to_valid_region) {
            min_u = 0;
            min_v = 0;
            max_u = full_height - 1;
            max_v = full_width - 1;
        }

        const uint32_t output_height = max_u - min_u + 1;
        const uint32_t output_width = max_v - min_v + 1;
        out.camera_info.roi.x_offset = min_v;
        out.camera_info.roi.y_offset = min_u;
        out.camera_info.roi.width = output_width;
        out.camera_info.roi.height = output_height;
        // The generated image is already rectified. The ROI is a crop in that
        // rectified full-panel coordinate system, so no separate rectified ROI
        // needs to be calculated.
        out.camera_info.roi.do_rectify = false;

        if (min_u == 0 && min_v == 0 && output_height == full_height &&
            output_width == full_width) {
            return;
        }

        const Eigen::Index row = static_cast<Eigen::Index>(min_u);
        const Eigen::Index col = static_cast<Eigen::Index>(min_v);
        const Eigen::Index rows = static_cast<Eigen::Index>(output_height);
        const Eigen::Index cols = static_cast<Eigen::Index>(output_width);
        out.r_src = out.r_src.block(row, col, rows, cols).eval();
        out.v_src = out.v_src.block(row, col, rows, cols).eval();
        out.raw_v_src = out.raw_v_src.block(row, col, rows, cols).eval();
        out.depth_scale = out.depth_scale.block(row, col, rows, cols).eval();
        out.depth_offset = out.depth_offset.block(row, col, rows, cols).eval();
    }

    bool raw_column_in_window(int32_t column) const {
        const auto& window = info_.format.column_window;
        return window.first <= window.second
                   ? column >= window.first && column <= window.second
                   : column >= window.first || column <= window.second;
    }

    double derive_lidar_vfov_rad(double panel_pitch_rad) const {
        const auto& alts = info_.beam_altitude_angles;
        if (alts.size() < 2) return M_PI / 2.0;  // 90-degree fallback
        auto [min_it, max_it] = std::minmax_element(alts.begin(), alts.end());
        const double min_elevation = *min_it * M_PI / 180.0;
        const double max_elevation = *max_it * M_PI / 180.0;
        const double normalized_pitch =
            std::remainder(panel_pitch_rad, 2.0 * M_PI);
        const double half_vfov = std::max(
            std::abs(max_elevation - normalized_pitch),
            std::abs(normalized_pitch - min_elevation));
        if (!std::isfinite(half_vfov) || half_vfov <= 0.0 ||
            half_vfov >= M_PI_2) {
            throw std::invalid_argument(
                "cannot auto-fit lidar VFOV around panel pitch");
        }
        return 2.0 * half_vfov;
    }

    static std::string substitute_template(const std::string& tmpl,
                                           const std::string& ns,
                                           const std::string& name) {
        std::string out = tmpl;
        for (auto& [key, value] :
             std::map<std::string, std::string>{{"{ns}", ns}, {"{name}", name}}) {
            size_t pos = 0;
            while ((pos = out.find(key, pos)) != std::string::npos) {
                out.replace(pos, key.size(), value);
                pos += value.size();
            }
        }
        while (!out.empty() && out.front() == '/') out.erase(0, 1);
        return out;
    }

    static bool profile_has_rgb(
        const ouster::sdk::core::SensorInfo& info) {
        using Profile = ouster::sdk::core::UDPProfileLidar;
        return info.format.udp_profile_lidar ==
                   Profile::RNG19_RFL8_SIG16_NIR16_RGB16 ||
               info.format.udp_profile_lidar ==
                   Profile::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL;
    }

    static std::map<std::string, std::string> channel_topic_map_for(
        int n_returns, bool has_rgb) {
        std::map<std::string, std::string> m {
            {ChanField::RANGE, "range_image"},
            {ChanField::SIGNAL, "signal_image"},
            {ChanField::REFLECTIVITY, "reflec_image"},
            {ChanField::NEAR_IR, "nearir_image"}};
        if (n_returns == 2) {
            m[ChanField::RANGE2] = "range_image2";
            m[ChanField::SIGNAL2] = "signal_image2";
            m[ChanField::REFLECTIVITY2] = "reflec_image2";
        }
        if (has_rgb) m[ChanField::RGB] = "rgb_image";
        return m;
    }

    static std::map<std::string, std::string> depth_topic_map_for(
        int n_returns) {
        std::map<std::string, std::string> m {
            {ChanField::RANGE, "depth_image"}};
        if (n_returns == 2) {
            m[ChanField::RANGE2] = "depth_image2";
        }
        return m;
    }

    void process(const ouster::sdk::core::LidarScan& scan, uint64_t,
                 const rclcpp::Time& msg_ts) {
        process_return(scan, /*return_index=*/0);
        if (info_.num_returns() == 2) process_return(scan, /*return_index=*/1);

        for (auto& panel : panels_) {
            for (auto& kv : panel->images) kv.second->header.stamp = msg_ts;
            for (auto& kv : panel->depth_images) {
                kv.second->header.stamp = msg_ts;
            }
            panel->camera_info.header.stamp = msg_ts;
        }
        if (post_processing_fn_) post_processing_fn_(panels_);
    }

    void process_return(const ouster::sdk::core::LidarScan& lidar_scan,
                        int return_index) {
        const bool first = return_index == 0;
        const uint32_t H = info_.format.pixels_per_column;
        const uint32_t W = info_.format.columns_per_frame;
        const auto& px_offset = info_.format.pixel_shift_by_row;

        const auto range_ch =
            first ? ChanField::RANGE : ChanField::RANGE2;
        const auto signal_ch =
            impl::scan_return(ChanField::SIGNAL, !first);
        const auto reflec_ch =
            impl::scan_return(ChanField::REFLECTIVITY, !first);
        const auto nearir_ch = ChanField::NEAR_IR;

        ouster::sdk::core::img_t<uint32_t> range =
            lidar_scan.field<uint32_t>(range_ch);
        ouster::sdk::core::img_t<uint32_t> signal =
            impl::get_or_fill_zero<uint32_t>(signal_ch, lidar_scan);
        ouster::sdk::core::img_t<uint16_t> reflec =
            impl::get_or_fill_zero<uint16_t>(reflec_ch, lidar_scan);
        ouster::sdk::core::img_t<uint16_t> nearir =
            (return_index == 0)
                ? impl::get_or_fill_zero<uint16_t>(nearir_ch, lidar_scan)
                : ouster::sdk::core::img_t<uint16_t>::Zero(H, W);

        // Auto-expose the full panorama so panels share a dynamic range.
        ouster::sdk::core::img_t<float> signal_f(H, W);
        ouster::sdk::core::img_t<float> reflec_f(H, W);
        ouster::sdk::core::img_t<float> nearir_f(H, W);
        ouster::sdk::core::img_t<pixel_type> range_dest(H, W);
        range_dest.setZero();

        // Match os_image: 4 mm radial range and zero beyond 16-bit capacity.
        const auto rg = range.data();
        const auto sg = signal.data();
        const auto rf = reflec.data();
        const auto nr = nearir.data();
        constexpr size_t pixel_value_max =
            std::numeric_limits<pixel_type>::max();

        for (size_t u = 0; u < H; ++u) {
            for (size_t v = 0; v < W; ++v) {
                const int32_t raw_v = destaggered_to_raw_column(
                    static_cast<int32_t>(v), px_offset[u],
                    static_cast<int32_t>(W));
                const size_t idx = u * W + static_cast<size_t>(raw_v);
                auto r = (rg[idx] + 0b10) >> 2;
                range_dest(u, v) = r > pixel_value_max ? 0 : r;
                signal_f(u, v) = static_cast<float>(sg[idx]);
                reflec_f(u, v) = static_cast<float>(rf[idx]);
                nearir_f(u, v) = static_cast<float>(nr[idx]);
            }
        }

        signal_ae_.update(signal_f, first);
        reflec_ae_.update(reflec_f, first);
        // NEAR_IR is shared by both returns; process it only once.
        if (first) {
            nearir_buc_.update(nearir_f);
            nearir_ae_.update(nearir_f, first);
            nearir_f = nearir_f.sqrt();
        }
        signal_f = signal_f.sqrt();

        ouster::sdk::core::img_t<pixel_type> signal_dest =
            (signal_f * pixel_value_max).cast<pixel_type>();
        ouster::sdk::core::img_t<pixel_type> reflec_dest =
            (reflec_f * pixel_value_max).cast<pixel_type>();
        ouster::sdk::core::img_t<pixel_type> nearir_dest =
            (nearir_f * pixel_value_max).cast<pixel_type>();

        // Do not overwrite the first return's NEAR_IR panel on return two.
        std::map<std::string, const ouster::sdk::core::img_t<pixel_type>*>
            channel_to_image{
                {range_ch, &range_dest},
                {signal_ch, &signal_dest},
                {reflec_ch, &reflec_dest}};
        if (first) channel_to_image[nearir_ch] = &nearir_dest;

        for (auto& panel : panels_) {
            for (const auto& chan_img : channel_to_image) {
                const std::string& chan = chan_img.first;
                auto img_it = panel->images.find(chan);
                if (img_it == panel->images.end()) continue;
                sample_panel(*img_it->second, *chan_img.second, *panel);
            }
            auto depth_it = panel->depth_images.find(range_ch);
            if (depth_it != panel->depth_images.end()) {
                sample_depth_panel(*depth_it->second, range, *panel);
            }
        }

        // LidarPacketHandler tone-maps native float16 color into these three
        // uint8 fields once per scan. RGB is common to both lidar returns, so
        // resample it only while processing the first return.
        if (first && has_rgb_) {
            const auto red = impl::get_or_fill_zero<uint8_t>(
                ChanField::R_U8, lidar_scan);
            const auto green = impl::get_or_fill_zero<uint8_t>(
                ChanField::G_U8, lidar_scan);
            const auto blue = impl::get_or_fill_zero<uint8_t>(
                ChanField::B_U8, lidar_scan);
            for (auto& panel : panels_) {
                const auto rgb_it = panel->images.find(ChanField::RGB);
                if (rgb_it != panel->images.end()) {
                    sample_rgb_panel(*rgb_it->second, red, green, blue,
                                     *panel);
                }
            }
        }
    }

    void sample_panel(sensor_msgs::msg::Image& out,
                      const ouster::sdk::core::img_t<pixel_type>& src,
                      const PanelOutput& panel) const {
        const uint32_t pw = out.width;
        const uint32_t ph = out.height;
        for (uint32_t u = 0; u < ph; ++u) {
            for (uint32_t v = 0; v < pw; ++v) {
                const int32_t r = panel.r_src(u, v);
                const int32_t c = panel.v_src(u, v);
                pixel_type val = 0;
                if (r >= 0) {
                    val = src(r, c);
                }
                const size_t output_offset =
                    (static_cast<size_t>(u) * pw + v) * sizeof(pixel_type);
                std::memcpy(out.data.data() + output_offset, &val,
                            sizeof(pixel_type));
            }
        }
    }

    void sample_rgb_panel(
        sensor_msgs::msg::Image& out,
        const ouster::sdk::core::img_t<uint8_t>& red,
        const ouster::sdk::core::img_t<uint8_t>& green,
        const ouster::sdk::core::img_t<uint8_t>& blue,
        const PanelOutput& panel) const {
        const uint32_t pw = out.width;
        const uint32_t ph = out.height;
        const int32_t W =
            static_cast<int32_t>(info_.format.columns_per_frame);
        for (uint32_t u = 0; u < ph; ++u) {
            for (uint32_t v = 0; v < pw; ++v) {
                std::array<uint8_t, 3> rgb{0, 0, 0};
                const int32_t r = panel.r_src(u, v);
                const int32_t raw_v = panel.raw_v_src(u, v);
                if (r >= 0 && raw_v >= 0) {
                    const size_t source_index =
                        static_cast<size_t>(r) * W + raw_v;
                    rgb = {red.data()[source_index],
                           green.data()[source_index],
                           blue.data()[source_index]};
                }
                const size_t output_offset =
                    (static_cast<size_t>(u) * pw + v) * rgb.size();
                std::memcpy(out.data.data() + output_offset, rgb.data(),
                            rgb.size());
            }
        }
    }

    void sample_depth_panel(
        sensor_msgs::msg::Image& out,
        const ouster::sdk::core::img_t<uint32_t>& raw_range,
        const PanelOutput& panel) const {
        const uint32_t pw = out.width;
        const uint32_t ph = out.height;
        const int32_t W = static_cast<int32_t>(info_.format.columns_per_frame);
        const auto* ranges = raw_range.data();
        const float invalid_depth =
            std::numeric_limits<float>::quiet_NaN();

        for (uint32_t u = 0; u < ph; ++u) {
            for (uint32_t v = 0; v < pw; ++v) {
                float depth = invalid_depth;
                const int32_t r = panel.r_src(u, v);
                if (r >= 0) {
                    const int32_t raw_v = panel.raw_v_src(u, v);
                    const size_t source_index =
                        static_cast<size_t>(r) * W + raw_v;
                    const uint32_t range = ranges[source_index];
                    if (range != 0) {
                        const float candidate =
                            static_cast<float>(range) *
                                panel.depth_scale(u, v) +
                            panel.depth_offset(u, v);
                        if (std::isfinite(candidate) && candidate > 0.0f) {
                            depth = candidate;
                        }
                    }
                }
                const size_t output_offset =
                    (static_cast<size_t>(u) * pw + v) * sizeof(float);
                std::memcpy(out.data.data() + output_offset, &depth,
                            sizeof(float));
            }
        }
    }

    static int32_t wrap_index(int64_t value, int32_t modulus) {
        int64_t wrapped = value % modulus;
        if (wrapped < 0) wrapped += modulus;
        return static_cast<int32_t>(wrapped);
    }

    static int32_t destaggered_to_raw_column(int32_t column, int shift,
                                              int32_t width) {
        return wrap_index(static_cast<int64_t>(column) - shift, width);
    }

    static int32_t raw_to_destaggered_column(int32_t column, int shift,
                                              int32_t width) {
        return wrap_index(static_cast<int64_t>(column) + shift, width);
    }

    static double normalize_columns(double value, uint32_t width) {
        if (!std::isfinite(value) || width == 0) {
            throw std::invalid_argument("invalid azimuth offset or width");
        }
        const double period = static_cast<double>(width);
        double normalized = std::fmod(value, period);
        if (normalized < 0.0) normalized += period;
        return normalized;
    }

    static double normalize_radians(double value) {
        double normalized = std::fmod(value, 2.0 * M_PI);
        if (normalized < 0.0) normalized += 2.0 * M_PI;
        return normalized;
    }

    static void validate_sensor_info(
        const ouster::sdk::core::SensorInfo& info) {
        const auto H = info.format.pixels_per_column;
        const auto W = info.format.columns_per_frame;
        if (H == 0 || W == 0 ||
            W > static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
            throw std::invalid_argument("invalid lidar image dimensions");
        }
        if (info.format.pixel_shift_by_row.size() != H) {
            throw std::invalid_argument(
                "pixel_shift_by_row must match lidar image height");
        }
        const auto& column_window = info.format.column_window;
        if (column_window.first < 0 || column_window.second < 0 ||
            column_window.first >= static_cast<int>(W) ||
            column_window.second >= static_cast<int>(W)) {
            throw std::invalid_argument(
                "column_window must lie inside the lidar image width");
        }
        if (info.beam_altitude_angles.size() != H ||
            info.beam_azimuth_angles.size() != H) {
            throw std::invalid_argument(
                "pinhole panels require one beam angle per lidar row");
        }
        const auto finite = [](const auto& values) {
            return std::all_of(values.begin(), values.end(),
                               [](double value) {
                                   return std::isfinite(value);
                               });
        };
        if (!finite(info.beam_altitude_angles) ||
            !finite(info.beam_azimuth_angles) ||
            !info.beam_to_lidar_transform.allFinite()) {
            throw std::invalid_argument("lidar calibration must be finite");
        }
    }

    static ouster::sdk::core::XYZLut make_lidar_xyz_lut(
        const ouster::sdk::core::SensorInfo& info) {
        validate_sensor_info(info);
        return ouster::sdk::core::make_xyz_lut(
            info.format.columns_per_frame, info.format.pixels_per_column,
            ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
            ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
            info.beam_altitude_angles);
    }

   private:
    ouster::sdk::core::SensorInfo info_;
    double azimuth_offset_rad_;
    PostProcessingFn post_processing_fn_;
    bool has_rgb_;
    std::vector<std::shared_ptr<PanelOutput>> panels_;

    ouster::sdk::core::image::AutoExposure signal_ae_;
    ouster::sdk::core::image::AutoExposure reflec_ae_;
    ouster::sdk::core::image::AutoExposure nearir_ae_;
    ouster::sdk::core::image::BeamUniformityCorrector nearir_buc_;
};

}  // namespace ouster_ros
