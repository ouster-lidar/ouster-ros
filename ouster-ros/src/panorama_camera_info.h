// Copyright 2026 John Cameron Furey
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <ouster/types.h>
#include <sensor_msgs/msg/camera_info.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace ouster_ros {

struct PanoramaCameraInfoResult {
    sensor_msgs::msg::CameraInfo camera_info;
    bool used_vertical_fallback = false;
    bool used_full_roi_fallback = false;
};

inline PanoramaCameraInfoResult make_panorama_camera_info(
    const ouster::sdk::core::SensorInfo& sensor_info,
    const std::string& frame_id) {
    const uint32_t height = sensor_info.format.pixels_per_column;
    const uint32_t width = sensor_info.format.columns_per_frame;
    if (height == 0 || width == 0 || frame_id.empty()) {
        throw std::invalid_argument("invalid panorama CameraInfo geometry");
    }

    PanoramaCameraInfoResult result;
    auto& camera_info = result.camera_info;
    const double fx = static_cast<double>(width) / (2.0 * M_PI);
    const double cx = static_cast<double>(width) / 2.0;

    double fy = 0.0;
    double cy = 0.0;
    const auto& altitudes = sensor_info.beam_altitude_angles;
    if (altitudes.size() >= 2 && height > 1) {
        const auto [min_altitude, max_altitude] =
            std::minmax_element(altitudes.begin(), altitudes.end());
        const double vfov =
            (*max_altitude - *min_altitude) * M_PI / 180.0;
        if (vfov > 0.0) {
            fy = static_cast<double>(height - 1) / vfov;
            cy = *max_altitude * M_PI / 180.0 * fy;
        }
    }
    if (fy == 0.0) {
        result.used_vertical_fallback = true;
        fy = static_cast<double>(height) / (2.0 * M_PI);
        cy = static_cast<double>(height) / 2.0;
    }

    camera_info.header.frame_id = frame_id;
    camera_info.height = height;
    camera_info.width = width;
    camera_info.distortion_model = "plumb_bob";
    camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0};

    const auto set_full_roi = [&camera_info, width, height]() {
        camera_info.roi.x_offset = 0;
        camera_info.roi.y_offset = 0;
        camera_info.roi.width = width;
        camera_info.roi.height = height;
    };

    const auto& column_window = sensor_info.format.column_window;
    const bool full_column_window =
        column_window.first == 0 &&
        column_window.second == static_cast<int>(width) - 1;
    if (full_column_window) {
        set_full_roi();
    } else {
        int min_shift = 0;
        int max_shift = 0;
        const auto& shifts = sensor_info.format.pixel_shift_by_row;
        if (!shifts.empty()) {
            const auto [min_value, max_value] =
                std::minmax_element(shifts.begin(), shifts.end());
            min_shift = *min_value;
            max_shift = *max_value;
        }

        const int x0 = column_window.first + min_shift;
        const int x1 = column_window.second + max_shift;
        if (column_window.first <= column_window.second && x0 >= 0 &&
            x1 < static_cast<int>(width)) {
            camera_info.roi.x_offset = static_cast<uint32_t>(x0);
            camera_info.roi.y_offset = 0;
            camera_info.roi.width = static_cast<uint32_t>(x1 - x0 + 1);
            camera_info.roi.height = height;
        } else {
            result.used_full_roi_fallback = true;
            set_full_roi();
        }
    }
    camera_info.roi.do_rectify = false;
    return result;
}

}  // namespace ouster_ros
