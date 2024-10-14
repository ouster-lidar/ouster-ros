/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file point_cloud_producer.h
 * @brief takes in a lidar scan object and produces a PointCloud2 message
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <sensor_msgs/image_encodings.h>

#include "ouster/image_processing.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;
using sensor::ChanField;

class ImageProcessor {
   public:
    using OutputType =
        std::map<sensor::ChanField, std::shared_ptr<sensor_msgs::Image>>;
    using PostProcessingFn = std::function<void(OutputType)>;

   public:
    ImageProcessor(const ouster::sensor::sensor_info& info,
                   const std::string& frame_id, PostProcessingFn func)
        : frame(frame_id), post_processing_fn(func), info_(info) {
        uint32_t H = info.format.pixels_per_column;
        uint32_t W = info.format.columns_per_frame;

        image_msgs[ChanField::RANGE] = create_image_msg<int32_t>(
            H, W, frame, sensor_msgs::image_encodings::TYPE_32SC1);
        image_msgs[ChanField::SIGNAL] = create_image_msg<uint16_t>(
            H, W, frame, sensor_msgs::image_encodings::MONO16);
        image_msgs[ChanField::REFLECTIVITY] = create_image_msg<uint16_t>(
            H, W, frame, sensor_msgs::image_encodings::MONO16);
        image_msgs[ChanField::NEAR_IR] = create_image_msg<uint16_t>(
            H, W, frame, sensor_msgs::image_encodings::MONO16);
        if (get_n_returns(info) == 2) {
            image_msgs[ChanField::RANGE2] = create_image_msg<int32_t>(
                H, W, frame, sensor_msgs::image_encodings::TYPE_32SC1);
            image_msgs[ChanField::SIGNAL2] = create_image_msg<uint16_t>(
            H, W, frame, sensor_msgs::image_encodings::MONO16);
            image_msgs[ChanField::REFLECTIVITY2] = create_image_msg<uint16_t>(
            H, W, frame, sensor_msgs::image_encodings::MONO16);
        }
    }

   private:
    template <typename pixel_type>
    static std::shared_ptr<sensor_msgs::Image> create_image_msg(
        size_t H, size_t W,
        const std::string& frame,
        const std::string& encoding) {
        auto msg = std::make_shared<sensor_msgs::Image>();
        msg->height = H;
        msg->width = W;
        msg->step = W * sizeof(pixel_type);
        msg->encoding = encoding;
        msg->data.resize(W * H * sizeof(pixel_type));
        msg->header.frame_id = frame;
        return msg;
    }

   private:
    void process(const ouster::LidarScan& lidar_scan, uint64_t,
                 const ros::Time& msg_ts) {
        process_return(lidar_scan, 0);
        if (get_n_returns(info_) == 2) process_return(lidar_scan, 1);
        for (auto it = image_msgs.begin(); it != image_msgs.end(); ++it) {
            it->second->header.stamp = msg_ts;
        }
        if (post_processing_fn) post_processing_fn(image_msgs);
    }

    void process_return(const ouster::LidarScan& lidar_scan, int return_index) {
        const bool first = return_index == 0;

        // across supported lidar profiles range is always 32-bit
        auto range_ch = first ? ChanField::RANGE : ChanField::RANGE2;
        ouster::img_t<uint32_t> range = lidar_scan.field<uint32_t>(range_ch);

        // NOTE: signal is 32-bit only in legacy and is 16-bit in the remaining profiles.
        auto signal_ch = impl::suitable_return(ChanField::SIGNAL, !first);
        ouster::img_t<uint32_t> signal = impl::get_or_fill_zero<uint32_t>(signal_ch, lidar_scan);

        // NOTE: reflectivity varies between 32-bit, 16-bit and 8-bit.
        auto reflect_ch = impl::suitable_return(ChanField::REFLECTIVITY, !first);
        ouster::img_t<uint16_t> reflectivity = impl::get_or_fill_zero<uint16_t>(reflect_ch, lidar_scan);

        // TODO: note that near_ir will be processed twice for DUAL return sensor
        ouster::img_t<uint16_t> near_ir = impl::get_or_fill_zero<uint16_t>(ChanField::NEAR_IR, lidar_scan);

        uint32_t H = info_.format.pixels_per_column;
        uint32_t W = info_.format.columns_per_frame;

        // views into message data
        auto range_image_map = Eigen::Map<ouster::img_t<uint32_t>>(
            (uint32_t*)image_msgs[range_ch]->data.data(), H, W);
        auto signal_image_map = Eigen::Map<ouster::img_t<uint16_t>>(
            (uint16_t*)image_msgs[signal_ch]->data.data(), H, W);
        auto reflec_image_map = Eigen::Map<ouster::img_t<uint16_t>>(
            (uint16_t*)image_msgs[reflect_ch]->data.data(), H, W);
        auto nearir_image_map = Eigen::Map<ouster::img_t<uint16_t>>(
            (uint16_t*)image_msgs[ChanField::NEAR_IR]->data.data(), H, W);

        const auto& px_offset = info_.format.pixel_shift_by_row;

        ouster::img_t<float> signal_image_eigen(H, W);
        ouster::img_t<float> reflec_image_eigen(H, W);
        ouster::img_t<float> nearir_image_eigen(H, W);

        const auto rg = range.data();
        const auto sg = signal.data();
        const auto rf = reflectivity.data();
        const auto nr = near_ir.data();

        // copy data out of LidarScan with destaggering
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const size_t idx = u * W + vv;
                range_image_map(u, v) = rg[idx];
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

        // TODO[UN]: we should have a custom scale per image type
        const size_t max_px = std::numeric_limits<uint16_t>::max();
        signal_image_map = (signal_image_eigen * max_px).cast<uint16_t>();
        reflec_image_map = (reflec_image_eigen * max_px).cast<uint16_t>();
        nearir_image_map = (nearir_image_eigen * max_px).cast<uint16_t>();
    }

   public:
    static LidarScanProcessor create(const ouster::sensor::sensor_info& info,
                                     const std::string& frame,
                                     PostProcessingFn func) {
        auto handler = std::make_shared<ImageProcessor>(info, frame, func);
        return [handler](const ouster::LidarScan& lidar_scan, uint64_t scan_ts,
                         const ros::Time& msg_ts) {
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
};

}  // namespace ouster_ros