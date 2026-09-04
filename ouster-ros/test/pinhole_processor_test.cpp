// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
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
    config.width = 3;
    config.height = 1;

    PinholeProcessor processor(info, {config}, "{ns}/{name}", "/lidar0",
                               0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    // The center ray maps to W - 0.25 columns. Rounding lands on W, which
    // must wrap to column zero rather than clamp to the last column.
    EXPECT_EQ(panel.v_src(0, 1), 0);
    EXPECT_EQ(panel.optical_frame_id, "lidar0/seam");
    EXPECT_EQ(panel.camera_info.width, 3u);
    EXPECT_EQ(panel.camera_info.height, 1u);
}

TEST(PinholeProcessorTest, ReportsPixelCenteredIndependentFovIntrinsics) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "configured";
    config.hfov_rad = M_PI_2;
    config.vfov_rad = M_PI / 3.0;
    config.width = 8;
    config.height = 6;
    config.crop_to_valid_region = false;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    EXPECT_DOUBLE_EQ(panel.camera_info.k[0], 4.0);
    EXPECT_NEAR(panel.camera_info.k[4], 3.0 / std::tan(M_PI / 6.0),
                1e-12);
    EXPECT_DOUBLE_EQ(panel.camera_info.k[2], 3.5);
    EXPECT_DOUBLE_EQ(panel.camera_info.k[5], 2.5);
    EXPECT_EQ(panel.camera_info.p[0], panel.camera_info.k[0]);
    EXPECT_EQ(panel.camera_info.p[5], panel.camera_info.k[4]);
    EXPECT_EQ(panel.camera_info.p[2], panel.camera_info.k[2]);
    EXPECT_EQ(panel.camera_info.p[6], panel.camera_info.k[5]);
    EXPECT_EQ(panel.camera_info.roi.x_offset, 0u);
    EXPECT_EQ(panel.camera_info.roi.y_offset, 0u);
    EXPECT_EQ(panel.camera_info.roi.width, 8u);
    EXPECT_EQ(panel.camera_info.roi.height, 6u);
    ASSERT_FALSE(panel.images.empty());
    EXPECT_EQ(panel.images.begin()->second->width, 8u);
    EXPECT_EQ(panel.images.begin()->second->height, 6u);
}

TEST(PinholeProcessorTest, CropsToSourceSupportAndReportsFullPanelRoi) {
    auto info = load_test_info();
    const int width = static_cast<int>(info.format.columns_per_frame);
    info.format.column_window = {width / 2 - 24, width / 2 + 24};

    PinholeProcessor::PanelConfig config;
    config.name = "rear_crop";
    config.yaw_rad = M_PI;
    config.width = 80;
    config.height = 16;
    config.crop_to_valid_region = true;

    PinholeProcessor::OutputType output;
    auto process = PinholeProcessor::create(
        info, {config}, "{name}", "", 0.0,
        [&output](PinholeProcessor::OutputType& panels) { output = panels; });
    ouster::sdk::core::LidarScan scan(info);
    scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE)
        .setConstant(10000);
    scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2)
        .setConstant(20000);
    process(scan, 0, rclcpp::Time{int64_t{1}, RCL_ROS_TIME});

    ASSERT_EQ(output.size(), 1u);
    const auto& panel = *output.front();
    const auto& roi = panel.camera_info.roi;

    EXPECT_EQ(panel.camera_info.width, 80u);
    EXPECT_EQ(panel.camera_info.height, 16u);
    EXPECT_GT(roi.x_offset, 0u);
    EXPECT_LT(roi.width, panel.camera_info.width);
    EXPECT_LE(roi.x_offset + roi.width, panel.camera_info.width);
    EXPECT_LE(roi.y_offset + roi.height, panel.camera_info.height);
    ASSERT_FALSE(panel.images.empty());
    EXPECT_EQ(panel.images.begin()->second->width, roi.width);
    EXPECT_EQ(panel.images.begin()->second->height, roi.height);
    EXPECT_EQ(panel.r_src.cols(), static_cast<Eigen::Index>(roi.width));
    EXPECT_EQ(panel.r_src.rows(), static_cast<Eigen::Index>(roi.height));

    bool found_valid_pixel = false;
    for (Eigen::Index u = 0; u < panel.r_src.rows(); ++u) {
        for (Eigen::Index v = 0; v < panel.r_src.cols(); ++v) {
            if (panel.r_src(u, v) < 0) continue;
            found_valid_pixel = true;
            EXPECT_GE(panel.raw_v_src(u, v),
                      info.format.column_window.first);
            EXPECT_LE(panel.raw_v_src(u, v),
                      info.format.column_window.second);
        }
    }
    EXPECT_TRUE(found_valid_pixel);

    // image_geometry applies the ROI offset to the full-resolution K/P. The
    // resulting principal point is therefore expressed in emitted pixels.
    EXPECT_DOUBLE_EQ(panel.camera_info.k[2] - roi.x_offset,
                     0.5 * (panel.camera_info.width - 1) - roi.x_offset);
    EXPECT_DOUBLE_EQ(panel.camera_info.k[5] - roi.y_offset,
                     0.5 * (panel.camera_info.height - 1) - roi.y_offset);
}

TEST(PinholeProcessorTest, CropsAcrossAWrappedColumnWindow) {
    auto info = load_test_info();
    const int width = static_cast<int>(info.format.columns_per_frame);
    info.format.column_window = {width - 24, 24};

    PinholeProcessor::PanelConfig config;
    config.name = "front_wrap";
    config.width = 80;
    config.height = 16;
    config.crop_to_valid_region = true;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);
    EXPECT_LT(panel.camera_info.roi.width, panel.camera_info.width);

    for (Eigen::Index u = 0; u < panel.r_src.rows(); ++u) {
        for (Eigen::Index v = 0; v < panel.r_src.cols(); ++v) {
            if (panel.r_src(u, v) < 0) continue;
            const int32_t raw_v = panel.raw_v_src(u, v);
            EXPECT_TRUE(raw_v >= info.format.column_window.first ||
                        raw_v <= info.format.column_window.second);
        }
    }
}

TEST(PinholeProcessorTest, CanKeepPaddingWithoutClaimingAnImageCrop) {
    auto info = load_test_info();
    const int width = static_cast<int>(info.format.columns_per_frame);
    info.format.column_window = {width / 2 - 24, width / 2 + 24};

    PinholeProcessor::PanelConfig config;
    config.name = "rear_padded";
    config.yaw_rad = M_PI;
    config.width = 80;
    config.height = 16;
    config.crop_to_valid_region = false;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);

    EXPECT_EQ(panel.camera_info.roi.x_offset, 0u);
    EXPECT_EQ(panel.camera_info.roi.y_offset, 0u);
    EXPECT_EQ(panel.camera_info.roi.width, panel.camera_info.width);
    EXPECT_EQ(panel.camera_info.roi.height, panel.camera_info.height);
    EXPECT_EQ(panel.r_src.cols(), 80);
    EXPECT_EQ(panel.r_src.rows(), 16);
    EXPECT_GT((panel.r_src < 0).count(), 0);
}

TEST(PinholeProcessorTest, CanKeepAWhollyUnsupportedPaddedPanel) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "sky";
    config.pitch_rad = 80.0 * M_PI / 180.0;
    config.width = 1;
    config.height = 1;
    config.crop_to_valid_region = false;

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    const auto& panel = *processor.panels().at(0);
    EXPECT_EQ(panel.camera_info.roi.width, 1u);
    EXPECT_EQ(panel.camera_info.roi.height, 1u);
    EXPECT_EQ(panel.r_src(0, 0), -1);

    config.crop_to_valid_region = true;
    EXPECT_THROW(PinholeProcessor(info, {config}, "{name}", "", 0.0,
                                  nullptr),
                 std::invalid_argument);
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

TEST(PinholeProcessorTest, AutoHeightCoversAsymmetricVfovAroundPitch) {
    const auto info = load_test_info();
    const auto [min_altitude, max_altitude] = std::minmax_element(
        info.beam_altitude_angles.begin(), info.beam_altitude_angles.end());

    PinholeProcessor::PanelConfig config;
    config.name = "pitched";
    config.pitch_rad = 10.0 * M_PI / 180.0;
    config.hfov_rad = M_PI_2;
    config.width = 256;
    config.height = 0;
    config.vfov_rad = 0.0;
    config.crop_to_valid_region = false;

    const double half_vfov = std::max(
        std::abs(*max_altitude * M_PI / 180.0 - config.pitch_rad),
        std::abs(config.pitch_rad - *min_altitude * M_PI / 180.0));
    const uint32_t expected_height = static_cast<uint32_t>(
        std::round(2.0 * 128.0 * std::tan(half_vfov)));

    PinholeProcessor processor(info, {config}, "{name}", "", 0.0, nullptr);
    EXPECT_EQ(processor.panels().front()->camera_info.height,
              expected_height);
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
        config.width = 5;
        config.height = 3;
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

TEST(PinholeProcessorTest, SelectedLidarPointsReprojectNearPanelPixels) {
    const auto info = load_test_info();

    PinholeProcessor::PanelConfig config;
    config.name = "oblique";
    config.yaw_rad = 0.37;
    config.pitch_rad = 0.08;
    config.hfov_rad = M_PI_2;
    config.vfov_rad = 50.0 * M_PI / 180.0;
    config.width = 257;
    config.height = 129;
    config.crop_to_valid_region = false;

    const double azimuth_offset_rad = 0.21;
    const double azimuth_offset_columns =
        azimuth_offset_rad * info.format.columns_per_frame / (2.0 * M_PI);
    PinholeProcessor processor(info, {config}, "{name}", "",
                               azimuth_offset_columns, nullptr);
    const auto& panel = *processor.panels().at(0);
    const auto xyz_lut = ouster::sdk::core::make_xyz_lut(
        info.format.columns_per_frame, info.format.pixels_per_column,
        ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
        ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
        info.beam_altitude_angles);

    // Compare in SDK lidar coordinates. The advertised parent frame rotates
    // SDK column zero by azimuth_offset_rad, so its panel yaw is offset here.
    const double lidar_yaw = config.yaw_rad - azimuth_offset_rad;
    const double cosy = std::cos(lidar_yaw);
    const double siny = std::sin(lidar_yaw);
    const double cosp = std::cos(config.pitch_rad);
    const double sinp = std::sin(config.pitch_rad);
    const Eigen::Vector3d optical_x{siny, -cosy, 0.0};
    const Eigen::Vector3d optical_y{cosy * sinp, siny * sinp, -cosp};
    const Eigen::Vector3d optical_z{cosy * cosp, siny * cosp, sinp};

    size_t checked = 0;
    for (const uint32_t u : {16u, 32u, 64u, 96u, 112u}) {
        for (const uint32_t v : {16u, 64u, 128u, 192u, 240u}) {
            const int32_t row = panel.r_src(u, v);
            const int32_t raw_col = panel.raw_v_src(u, v);
            if (row < 0 || raw_col < 0) continue;
            const Eigen::Index source_index =
                static_cast<Eigen::Index>(row) *
                    info.format.columns_per_frame +
                raw_col;
            const Eigen::Vector3d source_direction =
                xyz_lut.direction.row(source_index).matrix().transpose();
            const Eigen::Vector3d point =
                (10000.0 * xyz_lut.direction.row(source_index) +
                 xyz_lut.offset.row(source_index))
                    .matrix()
                    .transpose();
            const double optical_depth = optical_z.dot(point);
            ASSERT_GT(optical_depth, 0.0);
            const double projected_u =
                panel.camera_info.k[0] * optical_x.dot(point) /
                    optical_depth +
                panel.camera_info.k[2];
            const double projected_v =
                panel.camera_info.k[4] * optical_y.dot(point) /
                    optical_depth +
                panel.camera_info.k[5];
            EXPECT_NEAR(projected_u, static_cast<double>(v), 1.0);
            EXPECT_NEAR(projected_v, static_cast<double>(u), 1.0);
            EXPECT_NEAR(panel.depth_scale(u, v),
                        optical_z.dot(source_direction), 1e-7);
            ++checked;
        }
    }
    EXPECT_GE(checked, 10u);
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
