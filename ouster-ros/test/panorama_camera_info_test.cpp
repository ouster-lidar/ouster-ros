// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include "ouster_ros/impl/file_util.h"
#include "../src/panorama_camera_info.h"

namespace ouster_ros {
namespace {

ouster::sdk::core::SensorInfo load_panorama_test_info() {
    const auto metadata =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "ouster-sdk/tests/metadata/3_0_1_os-122246000293-128.json";
    return ouster::sdk::core::SensorInfo(
        impl::read_text_file(metadata.string()));
}

TEST(PanoramaCameraInfoTest, UsesBeamAltitudeIntrinsicsAndFullRoi) {
    const auto info = load_panorama_test_info();
    const auto result =
        make_panorama_camera_info(info, "os_lidar_panorama_optical_frame");
    const auto& camera_info = result.camera_info;

    const auto [min_altitude, max_altitude] = std::minmax_element(
        info.beam_altitude_angles.begin(), info.beam_altitude_angles.end());
    const double vfov = (*max_altitude - *min_altitude) * M_PI / 180.0;
    const double expected_fy =
        static_cast<double>(info.format.pixels_per_column - 1) / vfov;

    EXPECT_FALSE(result.used_vertical_fallback);
    EXPECT_FALSE(result.has_partial_column_window);
    EXPECT_EQ(camera_info.header.frame_id,
              "os_lidar_panorama_optical_frame");
    EXPECT_DOUBLE_EQ(camera_info.k[0],
                     info.format.columns_per_frame / (2.0 * M_PI));
    EXPECT_DOUBLE_EQ(camera_info.k[4], expected_fy);
    EXPECT_DOUBLE_EQ(camera_info.k[5],
                     *max_altitude * M_PI / 180.0 * expected_fy);
    EXPECT_EQ(camera_info.roi.x_offset, 0u);
    EXPECT_EQ(camera_info.roi.width, info.format.columns_per_frame);
    EXPECT_EQ(camera_info.roi.height, info.format.pixels_per_column);
}

TEST(PanoramaCameraInfoTest, KeepsFullRoiForZeroPaddedPartialWindow) {
    auto info = load_panorama_test_info();
    info.format.column_window = {100, 200};
    info.format.pixel_shift_by_row.assign(
        info.format.pixels_per_column, 10);

    const auto result = make_panorama_camera_info(info, "optical_frame");

    EXPECT_TRUE(result.has_partial_column_window);
    EXPECT_EQ(result.camera_info.roi.x_offset, 0u);
    EXPECT_EQ(result.camera_info.roi.width, info.format.columns_per_frame);
    EXPECT_EQ(result.camera_info.roi.height, info.format.pixels_per_column);
}

TEST(PanoramaCameraInfoTest, KeepsFullRoiWhenPartialWindowWraps) {
    auto info = load_panorama_test_info();
    info.format.column_window = {
        static_cast<int>(info.format.columns_per_frame) - 20, 20};

    const auto result = make_panorama_camera_info(info, "optical_frame");

    EXPECT_TRUE(result.has_partial_column_window);
    EXPECT_EQ(result.camera_info.roi.x_offset, 0u);
    EXPECT_EQ(result.camera_info.roi.width, info.format.columns_per_frame);
}

TEST(PanoramaCameraInfoTest, FallsBackForMissingBeamAltitudes) {
    auto info = load_panorama_test_info();
    info.beam_altitude_angles.clear();

    const auto result = make_panorama_camera_info(info, "optical_frame");

    EXPECT_TRUE(result.used_vertical_fallback);
    EXPECT_DOUBLE_EQ(
        result.camera_info.k[4],
        info.format.pixels_per_column / (2.0 * M_PI));
    EXPECT_DOUBLE_EQ(result.camera_info.k[5],
                     info.format.pixels_per_column / 2.0);
}

}  // namespace
}  // namespace ouster_ros
