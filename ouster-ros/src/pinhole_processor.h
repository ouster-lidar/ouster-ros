// Copyright 2026 John Cameron Furey
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @file pinhole_processor.h
 * @brief Resamples destaggered lidar images into pinhole panels.
 *
 * range_image retains os_image's radial-range encoding: uint16 with 4 mm per
 * increment. depth_image is calibrated optical-axis depth in metres (32FC1)
 * for standard ROS depth consumers.
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
        uint32_t height = 0;        // 0: auto-fit lidar VFOV at square pixels
        double vfov_rad = 0.0;      // used when height is zero
    };

    using pixel_type = uint16_t;

    struct PanelOutput {
        std::string name;
        std::string optical_frame_id;
        double yaw_rad = 0.0;
        double pitch_rad = 0.0;
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>> images;
        // Metric optical-axis depth images keyed by RANGE / RANGE2. These are
        // separate from images because the other outputs are uint16 display
        // images while depth is 32FC1 in metres.
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>>
            depth_images;
        sensor_msgs::msg::CameraInfo camera_info;
        // LUT in destaggered space:
        //   r_src(u, v) = source row (-1 means outside lidar VFOV)
        //   v_src(u, v) = source destaggered column
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> r_src;
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> v_src;
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
                     PostProcessingFn func)
        : info_(info),
          azimuth_offset_columns_(0.0),
          post_processing_fn_(func) {
        if (panel_configs.empty()) {
            throw std::invalid_argument("at least one panel is required");
        }
        const auto xyz_lut = make_lidar_xyz_lut(info_);
        azimuth_offset_columns_ = normalize_columns(
            azimuth_offset_columns, info_.format.columns_per_frame);
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

    static std::map<std::string, std::string> channel_topics(int n_returns) {
        return channel_topic_map_for(n_returns);
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
        PostProcessingFn func) {
        auto handler = std::make_shared<PinholeProcessor>(
            info, panel_configs, optical_frame_template, lidar_namespace,
            azimuth_offset_columns, func);
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
        const double fy = fx;  // square pixels

        uint32_t ph = cfg.height;
        if (ph == 0) {
            const double vfov_rad = (cfg.vfov_rad > 0.0)
                ? cfg.vfov_rad
                : derive_lidar_vfov_rad();
            double ph_d = std::round(2.0 * fy * std::tan(0.5 * vfov_rad));
            if (!std::isfinite(ph_d) || ph_d < 1.0) ph_d = 1.0;
            const uint64_t max_height = std::min<uint64_t>(
                MAX_PANEL_DIMENSION, MAX_PANEL_PIXELS / pw);
            if (ph_d > static_cast<double>(max_height)) {
                ph_d = static_cast<double>(max_height);
            }
            ph = static_cast<uint32_t>(ph_d);
        }

        const uint64_t panel_pixels = static_cast<uint64_t>(pw) * ph;
        if (panel_pixels == 0 || panel_pixels > MAX_PANEL_PIXELS) {
            throw std::invalid_argument("panel pixel count out of range");
        }
        if (total_panel_pixels > MAX_TOTAL_PANEL_PIXELS - panel_pixels) {
            throw std::invalid_argument("total panel pixel count out of range");
        }
        total_panel_pixels += panel_pixels;

        const double cx = static_cast<double>(pw) / 2.0;
        const double cy = static_cast<double>(ph) / 2.0;

        out->camera_info.header.frame_id = out->optical_frame_id;
        out->camera_info.width = pw;
        out->camera_info.height = ph;
        out->camera_info.distortion_model = "plumb_bob";
        out->camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
        out->camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
        out->camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
        out->camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0,
                              0.0, 0.0, 1.0, 0.0};
        out->camera_info.roi.x_offset = 0;
        out->camera_info.roi.y_offset = 0;
        out->camera_info.roi.width = pw;
        out->camera_info.roi.height = ph;
        out->camera_info.roi.do_rectify = false;

        const auto channel_topic_map = channel_topic_map_for(info_.num_returns());
        for (const auto& kv : channel_topic_map) {
            auto img = std::make_shared<sensor_msgs::msg::Image>();
            img->header.frame_id = out->optical_frame_id;
            img->width = pw;
            img->height = ph;
            img->encoding = sensor_msgs::image_encodings::MONO16;
            img->is_bigendian = 0;
            img->step = pw * sizeof(pixel_type);
            img->data.assign(static_cast<size_t>(img->step) * ph, 0);
            out->images[kv.first] = img;
        }

        const auto depth_topic_map = depth_topic_map_for(info_.num_returns());
        for (const auto& kv : depth_topic_map) {
            auto img = std::make_shared<sensor_msgs::msg::Image>();
            img->header.frame_id = out->optical_frame_id;
            img->width = pw;
            img->height = ph;
            img->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
            img->is_bigendian = 0;
            img->step = pw * sizeof(float);
            img->data.resize(static_cast<size_t>(img->step) * ph);
            const float invalid_depth =
                std::numeric_limits<float>::quiet_NaN();
            for (size_t offset = 0; offset < img->data.size();
                 offset += sizeof(float)) {
                std::memcpy(img->data.data() + offset, &invalid_depth,
                            sizeof(float));
            }
            out->depth_images[kv.first] = img;
        }

        compute_lut(*out, xyz_lut, fx, fy, cx, cy, pw, ph);
        return out;
    }

    void compute_lut(PanelOutput& out,
                     const ouster::sdk::core::XYZLut& xyz_lut, double fx,
                     double fy, double cx, double cy, uint32_t pw,
                     uint32_t ph) const {
        out.r_src.resize(ph, pw);
        out.v_src.resize(ph, pw);
        out.depth_scale.resize(ph, pw);
        out.depth_offset.resize(ph, pw);

        const auto& alts = info_.beam_altitude_angles;  // degrees, top to bottom
        const uint32_t H = info_.format.pixels_per_column;
        const uint32_t W = info_.format.columns_per_frame;

        const double cosy = std::cos(out.yaw_rad);
        const double siny = std::sin(out.yaw_rad);
        // Positive pitch tilts the panel's forward axis toward lidar +Z.
        const double cosp = std::cos(out.pitch_rad);
        const double sinp = std::sin(out.pitch_rad);
        const double forward_x = cosy * cosp;
        const double forward_y = siny * cosp;
        const double forward_z = sinp;

        // Permit one beam spacing at the vertical field-of-view boundary.
        double beam_spacing_deg = 0.0;
        if (alts.size() >= 2) {
            auto [min_it, max_it] =
                std::minmax_element(alts.begin(), alts.end());
            beam_spacing_deg = (*max_it - *min_it) /
                               static_cast<double>(alts.size() - 1);
        }
        const double snap_tol_deg = std::max(beam_spacing_deg, 0.5);

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
                if (best_diff > snap_tol_deg) r_src = -1;

                // Destaggered column index advances clockwise in lidar_frame:
                // col = (column_zero_azimuth - azimuth) * W / (2*pi).
                const double az_norm = normalize_radians(azimuth);
                const double v_dest = normalize_columns(
                    azimuth_offset_columns_ -
                        az_norm * static_cast<double>(W) / (2.0 * M_PI),
                    W);
                // v_dest is in [0, W) but rounding can land exactly on W;
                // wrap around the panorama seam instead of clamping to W-1.
                const int32_t v_src = wrap_index(
                    std::lround(v_dest), static_cast<int32_t>(W));

                const int32_t valid_r =
                    (r_src >= 0 && r_src < static_cast<int32_t>(H))
                        ? r_src
                        : -1;
                out.r_src(u, v) = valid_r;
                out.v_src(u, v) = v_src;

                if (valid_r < 0) {
                    out.depth_scale(u, v) =
                        std::numeric_limits<float>::quiet_NaN();
                    out.depth_offset(u, v) = 0.0f;
                    continue;
                }

                const int32_t raw_v = destaggered_to_raw_column(
                    v_src, info_.format.pixel_shift_by_row[valid_r],
                    static_cast<int32_t>(W));
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

    double derive_lidar_vfov_rad() const {
        const auto& alts = info_.beam_altitude_angles;
        if (alts.size() < 2) return M_PI / 2.0;  // 90-degree fallback
        auto [min_it, max_it] = std::minmax_element(alts.begin(), alts.end());
        return (*max_it - *min_it) * M_PI / 180.0;
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

    static std::map<std::string, std::string> channel_topic_map_for(
        int n_returns) {
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
                    const int32_t raw_v = destaggered_to_raw_column(
                        panel.v_src(u, v),
                        info_.format.pixel_shift_by_row[r], W);
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
    double azimuth_offset_columns_;
    PostProcessingFn post_processing_fn_;
    std::vector<std::shared_ptr<PanelOutput>> panels_;

    ouster::sdk::core::image::AutoExposure signal_ae_;
    ouster::sdk::core::image::AutoExposure reflec_ae_;
    ouster::sdk::core::image::AutoExposure nearir_ae_;
    ouster::sdk::core::image::BeamUniformityCorrector nearir_buc_;
};

}  // namespace ouster_ros
