/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_cloud_producer.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/image_encodings.hpp>

#include "ouster/image_processing.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;
using sensor::ChanField;

class ImageProcessor {
   public:
    using OutputType =
        std::map<ChanField, std::shared_ptr<sensor_msgs::msg::Image>>;
    using PostProcessingFn = std::function<void(OutputType)>;

   public:
    ImageProcessor(const ouster::sensor::sensor_info& info,
                   const std::string& frame_id,
                   const std::string& mask_path,
                   PostProcessingFn func)
        : frame(frame_id), post_processing_fn(func), info_(info) {
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;

        image_msgs[ChanField::RANGE] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::SIGNAL] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::REFLECTIVITY] = std::make_shared<sensor_msgs::msg::Image>();
        image_msgs[ChanField::NEAR_IR] = std::make_shared<sensor_msgs::msg::Image>();
        if (get_n_returns(info) == 2) {
            image_msgs[ChanField::RANGE2] =
                std::make_shared<sensor_msgs::msg::Image>();
            image_msgs[ChanField::SIGNAL2] =
                std::make_shared<sensor_msgs::msg::Image>();
            image_msgs[ChanField::REFLECTIVITY2] =
                std::make_shared<sensor_msgs::msg::Image>();
        }

        for (auto it = image_msgs.begin(); it != image_msgs.end(); ++it) {
            init_image_msg(*it->second, H, W, frame);
        }

        mask = impl::load_mask<pixel_type>(mask_path, H, W);
    }

   private:
    using pixel_type = uint16_t;
    const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

    static void init_image_msg(sensor_msgs::msg::Image& msg, size_t H, size_t W,
                               const std::string& frame) {
        msg.width = W;
        msg.height = H;
        msg.step = W * sizeof(pixel_type);
        // TODO: allow choosing higher image encoding representation
        msg.encoding = sensor_msgs::image_encodings::MONO16;
        msg.data.resize(W * H * sizeof(pixel_type));
        msg.header.frame_id = frame;
    }

   private:
    void process(const ouster::LidarScan& lidar_scan, uint64_t,
                 const rclcpp::Time& msg_ts) {
        process_return(lidar_scan, 0);
        if (get_n_returns(info_) == 2) process_return(lidar_scan, 1);
        for (auto it = image_msgs.begin(); it != image_msgs.end(); ++it) {
            it->second->header.stamp = msg_ts;
        }
        if (post_processing_fn) post_processing_fn(image_msgs);
    }

    // TODO: this functin could be benefit of some refactor
    void process_return(const ouster::LidarScan& lidar_scan, int return_index) {
        const bool first = return_index == 0;

        // across supported lidar profiles range is always 32-bit
        auto range_channel = first ? ChanField::RANGE : ChanField::RANGE2;
        ouster::img_t<uint32_t> range =
            lidar_scan.field<uint32_t>(range_channel);

        ouster::img_t<uint16_t> reflectivity = impl::get_or_fill_zero<uint16_t>(
            impl::scan_return(ChanField::REFLECTIVITY, !first),
            lidar_scan);

        ouster::img_t<uint32_t> signal = impl::get_or_fill_zero<uint32_t>(
            impl::scan_return(ChanField::SIGNAL, !first), lidar_scan);

        // TODO: note that near_ir will be processed twice for DUAL return
        // sensor
        ouster::img_t<uint16_t> near_ir = impl::get_or_fill_zero<uint16_t>(
            impl::scan_return(ChanField::NEAR_IR, !first), lidar_scan);

        uint32_t H = info_.format.pixels_per_column;
        uint32_t W = info_.format.columns_per_frame;

        // views into message data
        auto range_msg = image_msgs[impl::scan_return(ChanField::RANGE, !first)];
        auto range_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)range_msg->data.data(), H, W);
        auto signal_msg = image_msgs[impl::scan_return(ChanField::SIGNAL, !first)];
        auto signal_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)signal_msg->data.data(), H, W);
        auto reflectivity_msg = image_msgs[impl::scan_return(ChanField::REFLECTIVITY, !first)];
        auto reflec_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)reflectivity_msg->data.data(), H, W);
        auto near_ir_msg = image_msgs[impl::scan_return(ChanField::NEAR_IR, !first)];
        auto nearir_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)near_ir_msg->data.data(), H, W);

        const auto& px_offset = info_.format.pixel_shift_by_row;

        ouster::img_t<float> signal_image_eigen(H, W);
        ouster::img_t<float> reflec_image_eigen(H, W);
        ouster::img_t<float> nearir_image_eigen(H, W);

        const auto rg = range.data();
        const auto sg = signal.data();
        const auto rf = reflectivity.data();
        const auto nr = near_ir.data();

        // copy data out of Cloud message, with destaggering
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const size_t idx = u * W + vv;
                // TODO: re-examine this truncation later
                // 16 bit img: use 4mm resolution and throw out returns > 260m
                auto r = (rg[idx] + 0b10) >> 2;
                range_image_map(u, v) = r > pixel_value_max ? 0 : r;
                signal_image_eigen(u, v) = sg[idx];
                reflec_image_eigen(u, v) = rf[idx];
                nearir_image_eigen(u, v) = nr[idx];
            }
        }

        signal_ae(signal_image_eigen, first);
        reflec_ae(reflec_image_eigen, first);
        nearir_buc(nearir_image_eigen);
        nearir_ae(nearir_image_eigen, first);
        nearir_image_eigen = nearir_image_eigen.sqrt();
        signal_image_eigen = signal_image_eigen.sqrt();

        // copy data into image messages
        signal_image_map =
            (signal_image_eigen * pixel_value_max).cast<pixel_type>();
        reflec_image_map =
            (reflec_image_eigen * pixel_value_max).cast<pixel_type>();
        nearir_image_map =
            (nearir_image_eigen * pixel_value_max).cast<pixel_type>();

        if (mask.size() != 0) {
            range_image_map = range_image_map * mask;
            signal_image_map = signal_image_map * mask;
            reflec_image_map = reflec_image_map * mask;
            nearir_image_map = nearir_image_map * mask;
        }
    }

   public:
    static LidarScanProcessor create(const ouster::sensor::sensor_info& info,
                                     const std::string& frame,
                                     const std::string& mask_path,
                                     PostProcessingFn func) {
        auto handler = std::make_shared<ImageProcessor>(info, frame, mask_path, func);
        return [handler](const ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                         const rclcpp::Time& msg_ts) {
            handler->process(lidar_scan, scan_ts, msg_ts);
        };
    }

   private:
    std::string frame;
    OutputType image_msgs;
    PostProcessingFn post_processing_fn;
    sensor::sensor_info info_;

    viz::AutoExposure nearir_ae, signal_ae, reflec_ae;
    viz::BeamUniformityCorrector nearir_buc;

    ouster::img_t<pixel_type> mask;
};

}  // namespace ouster_ros