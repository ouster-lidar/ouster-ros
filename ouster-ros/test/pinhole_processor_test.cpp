// Copyright 2026 John Cameron Furey
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ouster_ros/impl/file_util.h"
#include "../src/pinhole_processor.h"

namespace ouster_ros {
namespace {

ouster::sdk::core::SensorInfo load_test_info(
    const std::string& filename =
        "3_0_1_os-122246000293-128.json") {
    const auto metadata =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "ouster-sdk/tests/metadata" / filename;
    return ouster::sdk::core::SensorInfo(
        impl::read_text_file(metadata.string()));
}

float depth_at(const sensor_msgs::msg::Image& image, uint32_t row,
               uint32_t column) {
    float value = 0.0f;
    const size_t offset =
        (static_cast<size_t>(row) * image.width + column) * sizeof(float);
    std::memcpy(&value, image.data.data() + offset, sizeof(float));
    return value;
}

uint16_t mono16_at(const sensor_msgs::msg::Image& image, uint32_t row,
                   uint32_t column) {
    uint16_t value = 0;
    const size_t offset =
        (static_cast<size_t>(row) * image.width + column) * sizeof(uint16_t);
    std::memcpy(&value, image.data.data() + offset, sizeof(uint16_t));
    return value;
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

TEST(PinholeProcessorTest, ProducesCalibratedMetricOpticalDepth) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;

    PinholeProcessor::OutputType output;
    auto process = PinholeProcessor::create(
        info, {config}, "{name}_optical_frame", "", 0.0,
        [&output](PinholeProcessor::OutputType& panels) { output = panels; });

    ouster::sdk::core::LidarScan scan(info);
    auto range = scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    auto range2 = scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2);
    range.setConstant(10000);
    range2.setConstant(20000);
    const rclcpp::Time stamp{int64_t{123456789}, RCL_ROS_TIME};
    process(scan, 0, stamp);

    ASSERT_EQ(output.size(), 1u);
    const auto& panel = *output.front();
    const auto range_it =
        panel.images.find(ouster::sdk::core::ChanField::RANGE);
    const auto depth_it =
        panel.depth_images.find(ouster::sdk::core::ChanField::RANGE);
    const auto depth2_it =
        panel.depth_images.find(ouster::sdk::core::ChanField::RANGE2);
    ASSERT_NE(range_it, panel.images.end());
    ASSERT_NE(depth_it, panel.depth_images.end());
    ASSERT_NE(depth2_it, panel.depth_images.end());
    ASSERT_TRUE(range_it->second);
    ASSERT_TRUE(depth_it->second);
    ASSERT_TRUE(depth2_it->second);
    const auto& depth = *depth_it->second;
    const auto& depth2 = *depth2_it->second;
    EXPECT_EQ(depth.encoding, sensor_msgs::image_encodings::TYPE_32FC1);
    EXPECT_EQ(depth.step, depth.width * sizeof(float));
    EXPECT_EQ(depth.header.frame_id, panel.camera_info.header.frame_id);
    EXPECT_EQ(rclcpp::Time(depth.header.stamp), stamp);
    EXPECT_EQ(rclcpp::Time(depth2.header.stamp), stamp);
    EXPECT_EQ(rclcpp::Time(panel.camera_info.header.stamp), stamp);

    constexpr uint32_t row = 1;
    constexpr uint32_t column = 2;
    const int32_t source_row = panel.r_src(row, column);
    ASSERT_GE(source_row, 0);
    const int32_t width = static_cast<int32_t>(info.format.columns_per_frame);
    int32_t raw_column =
        panel.v_src(row, column) -
        info.format.pixel_shift_by_row.at(source_row);
    raw_column %= width;
    if (raw_column < 0) raw_column += width;

    const auto xyz_lut = ouster::sdk::core::make_xyz_lut(
        info.format.columns_per_frame, info.format.pixels_per_column,
        ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
        ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
        info.beam_altitude_angles);
    const Eigen::Index source_index =
        static_cast<Eigen::Index>(source_row) * width + raw_column;
    const float expected_depth = static_cast<float>(
        10000.0 * xyz_lut.direction(source_index, 0) +
        xyz_lut.offset(source_index, 0));
    EXPECT_EQ(mono16_at(*range_it->second, row, column), 2500u);
    EXPECT_NEAR(depth_at(depth, row, column), expected_depth, 1e-5f);
    const float expected_depth2 = static_cast<float>(
        20000.0 * xyz_lut.direction(source_index, 0) +
        xyz_lut.offset(source_index, 0));
    EXPECT_NEAR(depth_at(depth2, row, column), expected_depth2, 1e-5f);

    range.setZero();
    process(scan, 0, stamp);
    EXPECT_TRUE(std::isnan(depth_at(depth, row, column)));
}

TEST(PinholeProcessorTest, NormalizesLargeAzimuthOffsets) {
    const auto info = load_test_info();
    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;

    constexpr double fractional_offset = 0.25;
    PinholeProcessor reference(info, {config}, "{name}", "",
                               fractional_offset, nullptr);
    const double repeated_offset =
        static_cast<double>(info.format.columns_per_frame) * 1000000.0 +
        fractional_offset;
    PinholeProcessor repeated(info, {config}, "{name}", "",
                              repeated_offset, nullptr);

    EXPECT_TRUE((reference.panels().front()->v_src ==
                 repeated.panels().front()->v_src)
                    .all());
}

TEST(PinholeProcessorTest, RejectsNonFiniteProjectionParameters) {
    const auto info = load_test_info();
    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;

    EXPECT_THROW(
        PinholeProcessor(info, {config}, "{name}", "",
                         std::numeric_limits<double>::infinity(), nullptr),
        std::invalid_argument);

    config.yaw_rad = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(PinholeProcessor(info, {config}, "{name}", "", 0.0,
                                  nullptr),
                 std::invalid_argument);
}

TEST(PinholeProcessorTest, RejectsIncompatibleLidarGeometry) {
    auto info = load_test_info();
    info.format.pixel_shift_by_row.pop_back();

    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;

    EXPECT_THROW(PinholeProcessor(info, {config}, "{name}", "", 0.0,
                                  nullptr),
                 std::invalid_argument);
}

TEST(PinholeProcessorTest, SupportsRepresentativeLegacyMetadata) {
    const std::vector<std::string> filenames{
        "1_14_beta_os1-991937000062-16A0_legacy.json",
        "1_14_6cccd_os-882002000138-32U0_legacy.json",
        "2_2_os-992119000444-128_legacy.json",
    };

    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;

    for (const auto& filename : filenames) {
        SCOPED_TRACE(filename);
        const auto info = load_test_info(filename);
        EXPECT_NO_THROW(
            PinholeProcessor(info, {config}, "{name}", "", 0.0, nullptr));
    }
}

}  // namespace
}  // namespace ouster_ros
