/**
 * @file pinhole_processor.h
 * @brief Resamples the destaggered Ouster panorama into N pinhole panels at
 *        configurable cardinal yaws (front/left/rear/right). Each panel is a
 *        rectified perspective view with optical-convention frame_id +
 *        CameraInfo, suitable for direct rendering in Foxglove/Trillium and
 *        for traditional image-pipeline consumers (image_proc, YOLO, etc.)
 *        that expect a pinhole camera.
 *
 * Built once per metadata callback: a per-output-pixel LUT mapping
 * (panel_row, panel_col) → (source_row, source_destaggered_col). Per-scan
 * work is field-decode + auto-exposure (full panorama) + LUT-sample into
 * the panel images. Auto-exposure runs on the full panorama so all panels
 * share a consistent dynamic range.
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
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "ouster/lidar_scan.h"
#include "ouster/image_processing.h"
#include "lidar_packet_handler.h"

namespace ouster_ros {

namespace ChanField = ouster::sdk::core::ChanField;

class PinholeProcessor {
   public:
    struct PanelConfig {
        std::string name;
        double yaw_rad = 0.0;       // CCW about lidar_frame +Z
        double pitch_rad = 0.0;     // up positive (about panel-yawed +Y)
        double hfov_rad = M_PI / 2.0;
        uint32_t width = 256;
        uint32_t height = 0;        // 0 → auto-fit lidar VFOV at square pixels
        double vfov_rad = 0.0;      // override if both height==0 and vfov_rad>0
    };

    using pixel_type = uint16_t;

    struct PanelOutput {
        std::string name;
        std::string optical_frame_id;
        // Geometry of this panel relative to lidar_frame parent.
        double yaw_rad = 0.0;
        double pitch_rad = 0.0;
        // Image messages (one per channel), reused across scans.
        std::map<std::string, std::shared_ptr<sensor_msgs::msg::Image>> images;
        // CameraInfo for this panel (optical convention pinhole).
        sensor_msgs::msg::CameraInfo camera_info;
        // LUT in destaggered space:
        //   r_src(u, v) = source row (-1 → outside lidar VFOV; pixel zeroed)
        //   v_src(u, v) = source destaggered column
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> r_src;
        Eigen::Array<int32_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> v_src;
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
          azimuth_offset_columns_(azimuth_offset_columns),
          post_processing_fn_(func) {
        // Strip leading slash for {ns} substitution.
        std::string ns = lidar_namespace;
        while (!ns.empty() && ns.front() == '/') ns.erase(0, 1);

        for (const auto& cfg : panel_configs) {
            auto panel = build_panel(cfg, ns, optical_frame_template);
            panels_.push_back(panel);
        }
    }

    const std::vector<std::shared_ptr<PanelOutput>>& panels() const {
        return panels_;
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
    // ── Build helpers ────────────────────────────────────────────────────
    std::shared_ptr<PanelOutput> build_panel(
        const PanelConfig& cfg, const std::string& ns,
        const std::string& optical_frame_template) {
        auto out = std::make_shared<PanelOutput>();
        out->name = cfg.name;
        out->yaw_rad = cfg.yaw_rad;
        out->pitch_rad = cfg.pitch_rad;
        out->optical_frame_id =
            substitute_template(optical_frame_template, ns, cfg.name);

        const uint32_t pw = cfg.width;
        // Pinhole intrinsics (square pixels). HFOV→fx, then height auto-fits.
        const double half_hfov = 0.5 * cfg.hfov_rad;
        const double fx = static_cast<double>(pw) / (2.0 * std::tan(half_hfov));
        const double fy = fx;  // square pixels

        // Auto-fit height from lidar VFOV unless caller forced one.
        uint32_t ph = cfg.height;
        if (ph == 0) {
            const double vfov_rad = (cfg.vfov_rad > 0.0)
                ? cfg.vfov_rad
                : derive_lidar_vfov_rad();
            ph = static_cast<uint32_t>(std::round(
                2.0 * fy * std::tan(0.5 * vfov_rad)));
            if (ph < 1) ph = 1;
        }

        const double cx = static_cast<double>(pw) / 2.0;
        const double cy = static_cast<double>(ph) / 2.0;

        // Build CameraInfo (pinhole, optical convention, no distortion).
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

        // Build channel image buffers (MONO16, reused across scans).
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

        compute_lut(*out, fx, fy, cx, cy, pw, ph);
        return out;
    }

    void compute_lut(PanelOutput& out, double fx, double fy, double cx,
                     double cy, uint32_t pw, uint32_t ph) const {
        out.r_src.resize(ph, pw);
        out.v_src.resize(ph, pw);

        const auto& alts = info_.beam_altitude_angles;  // degrees, top→bottom
        const uint32_t H = info_.format.pixels_per_column;
        const uint32_t W = info_.format.columns_per_frame;

        const double cosy = std::cos(out.yaw_rad);
        const double siny = std::sin(out.yaw_rad);
        // Pitch convention: positive pitch_rad → panel looks UP. In
        // panel-body coords (post-b2o, pre-yaw) this is a rotation that
        // tilts the +X (forward) axis toward +Z (up):
        //   pb' = R_y(-pitch) * pb
        //       = (cos·pb_x - sin·pb_z,  pb_y,  sin·pb_x + cos·pb_z)
        const double cosp = std::cos(out.pitch_rad);
        const double sinp = std::sin(out.pitch_rad);

        // Tolerance for "elevation outside lidar VFOV" check, in degrees.
        // One beam spacing (~ vfov / (H-1)) seems generous enough to allow
        // the outermost rows to sample safely without snapping wildly.
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
                // Direction in panel optical frame.
                const double x_o = (static_cast<double>(v) - cx) / fx;
                const double y_o = (static_cast<double>(u) - cy) / fy;
                const double z_o = 1.0;

                // Optical → panel-body (rpy=-π/2,0,-π/2 is body→optical).
                //   body_x =  z_o ;  body_y = -x_o ;  body_z = -y_o.
                const double pb_x = z_o;
                const double pb_y = -x_o;
                const double pb_z = -y_o;

                // Pitch: rotate panel-body about +Y by -pitch (so positive
                // pitch tilts the +X "forward" axis toward +Z "up").
                const double pp_x = cosp * pb_x - sinp * pb_z;
                const double pp_y = pb_y;
                const double pp_z = sinp * pb_x + cosp * pb_z;

                // Pitched panel-body → lidar_frame: yaw φ about +Z (CCW).
                const double lx = cosy * pp_x - siny * pp_y;
                const double ly = siny * pp_x + cosy * pp_y;
                const double lz = pp_z;

                // Spherical in lidar_frame.
                const double azimuth = std::atan2(ly, lx);
                const double horiz = std::sqrt(lx * lx + ly * ly);
                const double elev_rad = std::atan2(lz, horiz);
                const double elev_deg = elev_rad * 180.0 / M_PI;

                // Elevation → nearest beam row.
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

                // Azimuth → destaggered column.
                // Ouster destagger advances columns with the firmware encoder.
                // The encoder rotates CCW about sensor +Z, but the URDF puts
                // lidar_frame as a 180° yaw of sensor_frame (lidar +X = cable
                // end), which flips the rotation sense in lidar_frame to CW.
                // So as col index k increases, lidar azimuth DECREASES:
                //     col k  ↔  lidar azimuth = col_0_az - k * 2π / W
                // For a target azimuth θ, the column whose content is at θ:
                //     col = (col_0_az - θ) * W / (2π)
                // azimuth_offset_columns_ is col_0_az converted to columns.
                double az_norm = azimuth;
                while (az_norm < 0.0) az_norm += 2.0 * M_PI;
                while (az_norm >= 2.0 * M_PI) az_norm -= 2.0 * M_PI;
                double v_dest = azimuth_offset_columns_ -
                                az_norm * static_cast<double>(W) /
                                (2.0 * M_PI);
                while (v_dest < 0.0) v_dest += static_cast<double>(W);
                while (v_dest >= static_cast<double>(W)) {
                    v_dest -= static_cast<double>(W);
                }
                int32_t v_src = static_cast<int32_t>(std::lround(v_dest));
                if (v_src < 0) v_src = 0;
                if (v_src >= static_cast<int32_t>(W)) v_src = W - 1;

                out.r_src(u, v) = (r_src >= 0 && r_src < static_cast<int32_t>(H))
                                      ? r_src
                                      : -1;
                out.v_src(u, v) = v_src;
            }
        }
    }

    double derive_lidar_vfov_rad() const {
        const auto& alts = info_.beam_altitude_angles;
        if (alts.size() < 2) return M_PI / 2.0;  // 90° fallback
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

    // ── Per-scan processing ──────────────────────────────────────────────
    void process(const ouster::sdk::core::LidarScan& scan, uint64_t,
                 const rclcpp::Time& msg_ts) {
        process_return(scan, /*return_index=*/0);
        if (info_.num_returns() == 2) process_return(scan, /*return_index=*/1);

        for (auto& panel : panels_) {
            for (auto& kv : panel->images) kv.second->header.stamp = msg_ts;
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

        // Channel field IDs (mirror image_processor.h).
        const auto range_ch =
            first ? ChanField::RANGE : ChanField::RANGE2;
        const auto signal_ch =
            impl::scan_return(ChanField::SIGNAL, !first);
        const auto reflec_ch =
            impl::scan_return(ChanField::REFLECTIVITY, !first);
        const auto nearir_ch = ChanField::NEAR_IR;

        // Decode raw fields (staggered).
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

        // Build full destaggered 16-bit panoramas (auto-exposed for the
        // signal/reflec/nearir channels). Auto-exposure runs on float
        // images, then we sample with the LUT into pinhole panels.
        ouster::sdk::core::img_t<float> signal_f(H, W);
        ouster::sdk::core::img_t<float> reflec_f(H, W);
        ouster::sdk::core::img_t<float> nearir_f(H, W);
        ouster::sdk::core::img_t<pixel_type> range_dest(H, W);
        range_dest.setZero();

        // First pass: destagger + cast/scale to float (for AE) and 16-bit
        // for range. Range conversion mirrors image_processor.h: 4mm
        // resolution, throw out > 260m (>= 65535).
        const auto rg = range.data();
        const auto sg = signal.data();
        const auto rf = reflec.data();
        const auto nr = nearir.data();
        constexpr size_t pixel_value_max =
            std::numeric_limits<pixel_type>::max();

        for (size_t u = 0; u < H; ++u) {
            for (size_t v = 0; v < W; ++v) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const size_t idx = u * W + vv;
                auto r = (rg[idx] + 0b10) >> 2;
                range_dest(u, v) = r > pixel_value_max ? 0 : r;
                signal_f(u, v) = static_cast<float>(sg[idx]);
                reflec_f(u, v) = static_cast<float>(rf[idx]);
                nearir_f(u, v) = static_cast<float>(nr[idx]);
            }
        }

        signal_ae_.update(signal_f, first);
        reflec_ae_.update(reflec_f, first);
        nearir_buc_.update(nearir_f);
        nearir_ae_.update(nearir_f, first);
        nearir_f = nearir_f.sqrt();
        signal_f = signal_f.sqrt();

        // Cast to 16-bit destaggered images (full panorama).
        ouster::sdk::core::img_t<pixel_type> signal_dest =
            (signal_f * pixel_value_max).cast<pixel_type>();
        ouster::sdk::core::img_t<pixel_type> reflec_dest =
            (reflec_f * pixel_value_max).cast<pixel_type>();
        ouster::sdk::core::img_t<pixel_type> nearir_dest =
            (nearir_f * pixel_value_max).cast<pixel_type>();

        // Sample each panel for each channel.
        const std::map<std::string, const ouster::sdk::core::img_t<pixel_type>*>
            channel_to_image{
                {range_ch, &range_dest},
                {signal_ch, &signal_dest},
                {reflec_ch, &reflec_dest},
                {nearir_ch, &nearir_dest}};

        for (auto& panel : panels_) {
            for (const auto& chan_img : channel_to_image) {
                const std::string& chan = chan_img.first;
                auto img_it = panel->images.find(chan);
                if (img_it == panel->images.end()) continue;
                sample_panel(*img_it->second, *chan_img.second, *panel);
            }
        }
    }

    void sample_panel(sensor_msgs::msg::Image& out,
                      const ouster::sdk::core::img_t<pixel_type>& src,
                      const PanelOutput& panel) const {
        const uint32_t pw = out.width;
        const uint32_t ph = out.height;
        pixel_type* dst = reinterpret_cast<pixel_type*>(out.data.data());
        for (uint32_t u = 0; u < ph; ++u) {
            for (uint32_t v = 0; v < pw; ++v) {
                const int32_t r = panel.r_src(u, v);
                const int32_t c = panel.v_src(u, v);
                pixel_type val = 0;
                if (r >= 0) {
                    val = src(r, c);
                }
                dst[u * pw + v] = val;
            }
        }
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
