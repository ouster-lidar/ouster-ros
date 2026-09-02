// Copyright 2026 John Cameron Furey
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster_ros/impl/file_util.h"
#include "../src/pinhole_processor.h"

namespace ouster_ros {
namespace {

ouster::sdk::core::SensorInfo load_test_info() {
    const auto metadata =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "ouster-sdk/tests/metadata/3_0_1_os-122246000293-128.json";
    return ouster::sdk::core::SensorInfo(
        impl::read_text_file(metadata.string()));
}

TEST(PinholeProcessorTest, WrapsRoundedColumnAtPanoramaSeam) {
    const auto info = load_test_info();
    const double quarter_column_azimuth =
        0.25 * 2.0 * M_PI /
        static_cast<double>(info.format.columns_per_frame);

    PinholeProcessor::PanelConfig config;
    config.name = "seam";
    config.yaw_rad = quarter_column_azimuth;
    config.width = 2;
    config.height = 1;

    PinholeProcessor processor(info, {config}, "{ns}/{name}", "/lidar0",
                               0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    // The center ray maps to W - 0.25 columns. Rounding lands on W, which
    // must wrap to column zero rather than clamp to the last column.
    EXPECT_EQ(panel.v_src(0, 1), 0);
    EXPECT_EQ(panel.optical_frame_id, "lidar0/seam");
    EXPECT_EQ(panel.camera_info.width, 2u);
    EXPECT_EQ(panel.camera_info.height, 1u);
}

TEST(PinholeProcessorTest, ClampsAutoHeightBeforeAllocating) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "narrow";
    config.hfov_rad = 1e-12;
    config.width = 1;
    config.height = 0;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    EXPECT_EQ(panel.camera_info.height, 8192u);
    EXPECT_EQ(panel.r_src.rows(), 8192);
    EXPECT_EQ(panel.r_src.cols(), 1);
}

TEST(PinholeProcessorTest, RejectsPanelAbovePixelBudget) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "oversized";
    config.width = PinholeProcessor::MAX_PANEL_DIMENSION;
    config.height =
        PinholeProcessor::MAX_PANEL_PIXELS / config.width + 1;

    EXPECT_THROW(
        PinholeProcessor(info, {config}, "{name}", "", 0.0, nullptr),
        std::invalid_argument);
}

TEST(PinholeProcessorTest, CardinalPanelCentersTrackLidarAxes) {
    const auto info = load_test_info();
    const std::vector<std::pair<std::string, double>> cardinal_panels{
        {"front", 0.0},
        {"left", M_PI_2},
        {"rear", M_PI},
        {"right", 3.0 * M_PI_2},
    };

    std::vector<PinholeProcessor::PanelConfig> configs;
    for (const auto& [name, yaw] : cardinal_panels) {
        PinholeProcessor::PanelConfig config;
        config.name = name;
        config.yaw_rad = yaw;
        config.width = 4;
        config.height = 2;
        configs.push_back(config);
    }

    PinholeProcessor processor(info, configs, "{ns}/{name}", "lidar0",
                               0.0, nullptr);

    const int32_t width =
        static_cast<int32_t>(info.format.columns_per_frame);
    const double column_angle = 2.0 * M_PI / static_cast<double>(width);
    for (size_t i = 0; i < cardinal_panels.size(); ++i) {
        const auto& panel = *processor.panels().at(i);
        const int32_t row = panel.r_src(1, 2);
        const int32_t destaggered_col = panel.v_src(1, 2);
        ASSERT_GE(row, 0);
        ASSERT_LT(row, static_cast<int32_t>(info.format.pixels_per_column));

        // SDK destaggering maps output column v to raw column v - shift.
        int32_t raw_col =
            destaggered_col - info.format.pixel_shift_by_row.at(row);
        raw_col %= width;
        if (raw_col < 0) raw_col += width;

        const double raw_azimuth =
            2.0 * M_PI - static_cast<double>(raw_col) * column_angle -
            info.beam_azimuth_angles.at(row) * M_PI / 180.0;
        const double yaw_error =
            std::remainder(raw_azimuth - cardinal_panels[i].second,
                           2.0 * M_PI);

        // Integer destagger shifts and nearest-column sampling each contribute
        // at most roughly half a source column of angular error.
        EXPECT_NEAR(yaw_error, 0.0, column_angle + 1e-9);
        EXPECT_EQ(panel.optical_frame_id,
                  "lidar0/" + cardinal_panels[i].first);
        EXPECT_DOUBLE_EQ(panel.camera_info.k[0], panel.camera_info.k[4]);
    }
}

}  // namespace
}  // namespace ouster_ros
