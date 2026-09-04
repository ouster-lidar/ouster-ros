// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "ouster_ros/impl/file_util.h"
#include "../src/pinhole_processor.h"
#include "../src/point_cloud_processor_factory.h"

namespace ouster_ros {
namespace {

ouster::sdk::core::SensorInfo load_sdk_test_info(
    const std::filesystem::path& relative_path) {
    const auto metadata =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "ouster-sdk/tests" / relative_path;
    return ouster::sdk::core::SensorInfo(
        impl::read_text_file(metadata.string()));
}

ouster::sdk::core::SensorInfo load_test_info(
    const std::string& filename =
        "3_0_1_os-122246000293-128.json") {
    return load_sdk_test_info(std::filesystem::path("metadata") / filename);
}

ouster::sdk::core::SensorInfo make_synthetic_dome_info() {
    auto info = load_test_info();
    info.prod_line = "OS-DOME-128";
    info.format.column_window = {
        0, static_cast<int>(info.format.columns_per_frame) - 1};

    const size_t beam_count = info.format.pixels_per_column;
    if (beam_count != 128u) {
        throw std::logic_error("synthetic dome base metadata is not H128");
    }
    for (size_t row = 0; row < beam_count; ++row) {
        // Interleave low and high elevations to exercise the non-monotonic
        // row ordering used by dome-style calibration. This is deliberately
        // synthetic: the SDK checkout has no redistributable OSDome fixture.
        const size_t elevation_rank = row % 2 == 0
            ? row / 2
            : beam_count - 1 - row / 2;
        const double fraction = static_cast<double>(elevation_rank) /
                                static_cast<double>(beam_count - 1);
        info.beam_altitude_angles[row] = -1.0 + 88.0 * fraction;

        // Dome metadata can contain very large per-beam azimuth corrections.
        // Make the largest corrections occur near zenith, with both signs.
        const double magnitude = 2.0 + 98.0 * std::pow(fraction, 4.0);
        info.beam_azimuth_angles[row] =
            ((row / 2) % 2 == 0 ? 1.0 : -1.0) * magnitude;
        info.format.pixel_shift_by_row[row] =
            static_cast<int>((row * 73) % 571) - 285;
    }
    return info;
}

ouster::sdk::core::SensorInfo make_synthetic_rev8_max_info() {
    auto info = load_test_info();
    constexpr size_t beam_count = 256;
    constexpr uint32_t columns_per_frame = 4096;

    // The pinned SDK has the REV8 packet profiles but no redistributable REV8
    // metadata fixture yet. Model the published OS1 Max limits explicitly so
    // maximum dimensions and a 256-channel calibration remain covered without
    // presenting synthetic values as factory calibration.
    info.prod_line = "SYNTHETIC-OS1-MAX-256";
    info.format.pixels_per_column = beam_count;
    info.format.columns_per_frame = columns_per_frame;
    info.format.column_window = {
        0, static_cast<int>(columns_per_frame) - 1};
    info.format.udp_profile_lidar =
        ouster::sdk::core::UDPProfileLidar::
            RNG19_RFL8_SIG16_NIR16_RGB16_DUAL;
    info.beam_altitude_angles.resize(beam_count);
    info.beam_azimuth_angles.resize(beam_count);
    info.format.pixel_shift_by_row.resize(beam_count);

    for (size_t row = 0; row < beam_count; ++row) {
        const double fraction = static_cast<double>(row) /
                                static_cast<double>(beam_count - 1);
        info.beam_altitude_angles[row] = 21.95 - 43.9 * fraction;
        info.beam_azimuth_angles[row] =
            4.5 * std::sin(7.0 * M_PI * fraction);
        // Include negative shifts and values wider than a frame to exercise
        // the normalization contract rather than one firmware's value range.
        info.format.pixel_shift_by_row[row] =
            static_cast<int>((row * 109) % 10001) - 5000;
    }
    return info;
}

std::vector<PinholeProcessor::PanelConfig> dome_panel_configs(
    uint32_t dimension, bool crop_to_valid_region) {
    std::vector<PinholeProcessor::PanelConfig> configs;
    const std::array<const char*, 4> side_names{
        "front", "left", "rear", "right"};
    for (size_t i = 0; i < 4; ++i) {
        PinholeProcessor::PanelConfig config;
        config.name = side_names[i];
        config.yaw_rad = static_cast<double>(i) * M_PI_2;
        config.pitch_rad = 0.0;
        config.hfov_rad = 100.0 * M_PI / 180.0;
        config.vfov_rad = 100.0 * M_PI / 180.0;
        config.width = dimension;
        config.height = dimension;
        config.crop_to_valid_region = crop_to_valid_region;
        configs.push_back(config);
    }

    PinholeProcessor::PanelConfig top;
    top.name = "top";
    top.yaw_rad = 0.0;
    top.pitch_rad = M_PI_2;
    top.hfov_rad = 120.0 * M_PI / 180.0;
    top.vfov_rad = 120.0 * M_PI / 180.0;
    top.width = dimension;
    top.height = dimension;
    top.crop_to_valid_region = crop_to_valid_region;
    configs.push_back(top);
    return configs;
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

std::array<uint8_t, 3> rgb8_at(const sensor_msgs::msg::Image& image,
                               uint32_t row, uint32_t column) {
    const size_t offset =
        (static_cast<size_t>(row) * image.width + column) * 3;
    return {image.data[offset], image.data[offset + 1],
            image.data[offset + 2]};
}

Eigen::Matrix3d optical_to_lidar_rotation(double yaw_rad,
                                          double pitch_rad) {
    const double cosy = std::cos(yaw_rad);
    const double siny = std::sin(yaw_rad);
    const double cosp = std::cos(pitch_rad);
    const double sinp = std::sin(pitch_rad);

    Eigen::Matrix3d rotation;
    // Columns are the optical +X (right), +Y (down), and +Z (forward)
    // axes expressed in lidar coordinates.
    rotation.col(0) = Eigen::Vector3d{siny, -cosy, 0.0};
    rotation.col(1) =
        Eigen::Vector3d{cosy * sinp, siny * sinp, -cosp};
    rotation.col(2) =
        Eigen::Vector3d{cosy * cosp, siny * cosp, sinp};
    return rotation;
}

Eigen::Vector3d back_project_depth_pixel(
    const PinholeProcessor::PanelOutput& panel,
    const sensor_msgs::msg::Image& depth_image, uint32_t row,
    uint32_t column) {
    const double depth = depth_at(depth_image, row, column);
    const double full_column =
        static_cast<double>(column + panel.camera_info.roi.x_offset);
    const double full_row =
        static_cast<double>(row + panel.camera_info.roi.y_offset);
    return Eigen::Vector3d{
        (full_column - panel.camera_info.p[2]) * depth /
            panel.camera_info.p[0],
        (full_row - panel.camera_info.p[6]) * depth /
            panel.camera_info.p[5],
        depth};
}

std::vector<Eigen::Vector3f> point_cloud_xyz(
    const sensor_msgs::msg::PointCloud2& cloud) {
    std::vector<Eigen::Vector3f> points;
    points.reserve(static_cast<size_t>(cloud.width) * cloud.height);
    sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
    for (; x != x.end(); ++x, ++y, ++z) {
        points.emplace_back(*x, *y, *z);
    }
    return points;
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

TEST(PinholeProcessorTest,
     StandardDepthBackProjectionTracksButDoesNotDuplicateRawCloudPoints) {
    auto info = load_test_info();
    // Use the common dual-return profile so a synthetic LidarScan contains
    // every field consumed by the raw-cloud composition path.
    info.format.udp_profile_lidar =
        ouster::sdk::core::UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;
    const int32_t lidar_width =
        static_cast<int32_t>(info.format.columns_per_frame);
    // Force a crop in both image axes so the round trip also exercises the
    // CameraInfo ROI convention used by image_geometry/depth_image_proc.
    info.format.column_window =
        {lidar_width / 2 - 64, lidar_width / 2 + 64};

    PinholeProcessor::PanelConfig config;
    config.name = "rear_oblique";
    config.yaw_rad = M_PI + 0.13;
    config.pitch_rad = 0.08;
    config.hfov_rad = M_PI_2;
    config.vfov_rad = 50.0 * M_PI / 180.0;
    config.width = 257;
    config.height = 129;
    config.crop_to_valid_region = true;

    PinholeProcessor::OutputType panel_output;
    auto pinhole_process = PinholeProcessor::create(
        info, {config}, "{name}", "", 0.0,
        [&panel_output](PinholeProcessor::OutputType& panels) {
            panel_output = panels;
        });

    PointCloudProcessor_OutputType cloud_output;
    auto cloud_process =
        PointCloudProcessorFactory::create_point_cloud_processor(
            "xyz", info, "os_lidar", false, true, false, 0,
            std::numeric_limits<uint32_t>::max(), 1, "",
            [&cloud_output](PointCloudProcessor_OutputType clouds) {
                cloud_output = std::move(clouds);
            });

    PointCloudProcessor_OutputType destaggered_cloud_output;
    auto destaggered_cloud_process =
        PointCloudProcessorFactory::create_point_cloud_processor(
            "xyz", info, "os_lidar", false, true, true, 0,
            std::numeric_limits<uint32_t>::max(), 1, "",
            [&destaggered_cloud_output](PointCloudProcessor_OutputType clouds) {
                destaggered_cloud_output = std::move(clouds);
            });

    ouster::sdk::core::LidarScan scan(info);
    if (!scan.has_field(ouster::sdk::core::ChanField::WINDOW)) {
        scan.add_field({ouster::sdk::core::ChanField::WINDOW,
                        ouster::sdk::core::ChanFieldType::UINT8});
    }
    scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE)
        .setConstant(10000);
    scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2)
        .setConstant(20000);
    const rclcpp::Time stamp{int64_t{123456789}, RCL_ROS_TIME};
    cloud_process(scan, 0, stamp);
    destaggered_cloud_process(scan, 0, stamp);
    pinhole_process(scan, 0, stamp);

    ASSERT_EQ(panel_output.size(), 1u);
    ASSERT_EQ(cloud_output.size(), 2u);
    ASSERT_EQ(destaggered_cloud_output.size(), 2u);
    const auto& panel = *panel_output.front();
    EXPECT_GT(panel.camera_info.roi.x_offset, 0u);
    EXPECT_GT(panel.camera_info.roi.y_offset, 0u);
    EXPECT_LT(panel.camera_info.roi.width, panel.camera_info.width);
    EXPECT_LT(panel.camera_info.roi.height, panel.camera_info.height);
    const Eigen::Matrix3d optical_to_lidar =
        optical_to_lidar_rotation(config.yaw_rad, config.pitch_rad);

    for (size_t return_index = 0; return_index < cloud_output.size();
         ++return_index) {
        SCOPED_TRACE(return_index);
        ASSERT_TRUE(cloud_output[return_index]);
        ASSERT_TRUE(destaggered_cloud_output[return_index]);
        const auto raw_points = point_cloud_xyz(*cloud_output[return_index]);
        const auto destaggered_points =
            point_cloud_xyz(*destaggered_cloud_output[return_index]);
        ASSERT_EQ(raw_points.size(),
                  static_cast<size_t>(info.format.pixels_per_column) *
                      info.format.columns_per_frame);
        ASSERT_EQ(destaggered_points.size(), raw_points.size());

        size_t cloud_destagger_mismatches = 0;
        for (int32_t row = 0;
             row < static_cast<int32_t>(info.format.pixels_per_column);
             ++row) {
            const int32_t shift = info.format.pixel_shift_by_row[row];
            for (int32_t column = 0; column < lidar_width; ++column) {
                int32_t raw_column =
                    (column - shift) % lidar_width;
                if (raw_column < 0) raw_column += lidar_width;
                const size_t raw_index =
                    static_cast<size_t>(row) * lidar_width + raw_column;
                const size_t destaggered_index =
                    static_cast<size_t>(row) * lidar_width + column;
                if (!destaggered_points[destaggered_index].isApprox(
                        raw_points[raw_index], 1e-6f)) {
                    ++cloud_destagger_mismatches;
                }
            }
        }
        EXPECT_EQ(cloud_destagger_mismatches, 0u);

        const auto range_channel = return_index == 0
            ? ouster::sdk::core::ChanField::RANGE
            : ouster::sdk::core::ChanField::RANGE2;
        const auto depth_it = panel.depth_images.find(range_channel);
        ASSERT_NE(depth_it, panel.depth_images.end());
        ASSERT_TRUE(depth_it->second);
        const auto& depth_image = *depth_it->second;

        size_t checked = 0;
        size_t observably_displaced = 0;
        double squared_position_error_sum = 0.0;
        double max_position_error = 0.0;
        double max_optical_depth_error = 0.0;
        double max_reprojection_error = 0.0;
        std::unordered_set<size_t> unique_sources;
        for (uint32_t row = 0; row < depth_image.height; ++row) {
            for (uint32_t column = 0; column < depth_image.width; ++column) {
                const int32_t source_row = panel.r_src(row, column);
                const int32_t source_column =
                    panel.raw_v_src(row, column);
                if (source_row < 0 || source_column < 0) {
                    EXPECT_TRUE(std::isnan(
                        depth_at(depth_image, row, column)));
                    continue;
                }

                const size_t source_index =
                    static_cast<size_t>(source_row) *
                        info.format.columns_per_frame +
                    static_cast<size_t>(source_column);
                ASSERT_LT(source_index, raw_points.size());
                unique_sources.insert(source_index);
                const Eigen::Vector3d raw_lidar =
                    raw_points[source_index].cast<double>();
                const size_t destaggered_index =
                    static_cast<size_t>(source_row) *
                        info.format.columns_per_frame +
                    static_cast<size_t>(panel.v_src(row, column));
                ASSERT_LT(destaggered_index, destaggered_points.size());
                EXPECT_TRUE(destaggered_points[destaggered_index].isApprox(
                    raw_points[source_index], 1e-6f));
                ASSERT_TRUE(raw_lidar.allFinite());
                const Eigen::Vector3d raw_optical =
                    optical_to_lidar.transpose() * raw_lidar;
                ASSERT_GT(raw_optical.z(), 0.0);

                // This is the exact conversion performed by the standard ROS
                // XYZ depth node after image_geometry applies the ROI offset.
                const Eigen::Vector3d reconstructed_optical =
                    back_project_depth_pixel(panel, depth_image, row, column);
                ASSERT_TRUE(reconstructed_optical.allFinite());
                const Eigen::Vector3d reconstructed_lidar =
                    optical_to_lidar * reconstructed_optical;

                const double full_column = static_cast<double>(
                    column + panel.camera_info.roi.x_offset);
                const double full_row = static_cast<double>(
                    row + panel.camera_info.roi.y_offset);
                const double projected_column =
                    panel.camera_info.p[0] * raw_optical.x() /
                        raw_optical.z() +
                    panel.camera_info.p[2];
                const double projected_row =
                    panel.camera_info.p[5] * raw_optical.y() /
                        raw_optical.z() +
                    panel.camera_info.p[6];
                const double reprojection_error = std::hypot(
                    projected_column - full_column,
                    projected_row - full_row);
                const double position_error =
                    (reconstructed_lidar - raw_lidar).norm();
                const double optical_depth_error = std::abs(
                    reconstructed_optical.z() - raw_optical.z());

                max_reprojection_error =
                    std::max(max_reprojection_error, reprojection_error);
                max_position_error =
                    std::max(max_position_error, position_error);
                max_optical_depth_error =
                    std::max(max_optical_depth_error, optical_depth_error);
                squared_position_error_sum +=
                    position_error * position_error;
                if (position_error > 1e-3) ++observably_displaced;
                ++checked;
            }
        }

        ASSERT_GT(checked, 1000u);
        // Optical-axis Z is preserved to float precision. X/Y generally are
        // not: the selected lidar return usually projects between output pixel
        // centres, while a depth image can reconstruct only on the centre ray.
        EXPECT_LT(max_optical_depth_error, 5e-5);
        EXPECT_GT(observably_displaced, checked / 2);
        EXPECT_GT(max_position_error, 1e-3);
        EXPECT_LT(max_position_error, 0.5);
        EXPECT_LT(max_reprojection_error, 4.0);
        EXPECT_LT(unique_sources.size(), checked);

        const std::string suffix = std::to_string(return_index + 1);
        testing::Test::RecordProperty(
            "round_trip_max_error_m_return_" + suffix,
            max_position_error);
        testing::Test::RecordProperty(
            "round_trip_rms_error_m_return_" + suffix,
            std::sqrt(squared_position_error_sum /
                      static_cast<double>(checked)));
        testing::Test::RecordProperty(
            "round_trip_max_reprojection_px_return_" + suffix,
            max_reprojection_error);
        testing::Test::RecordProperty(
            "round_trip_unique_source_ratio_return_" + suffix,
            static_cast<double>(unique_sources.size()) /
                static_cast<double>(checked));
    }
}

TEST(PinholeProcessorTest,
     RoundTripGeometryAcrossBeamCountsProductsAndRangeDiscontinuities) {
    struct GeometryCase {
        const char* name;
        std::filesystem::path metadata;
    };
    const std::vector<GeometryCase> cases{
        {"os1_16_legacy",
         "metadata/1_14_beta_os1-991937000062-16A0_legacy.json"},
        {"os1_32_uniform_fw2_0",
         "metadata/2_0_rc2_os-992011000121-32U0_legacy.json"},
        {"os1_32_gradient_fw2_1",
         "pcaps/OS-1-32-G_v2.1.1_1024x10.json"},
        {"os1_64_fw1_12",
         "metadata/1_12_os1-991913000010-64.json"},
        {"os1_64_above_horizon_fw2_5",
         "pcaps/same_ports_nonlegacy.2.json"},
        {"os1_128_fw3_0",
         "metadata/3_0_1_os-122246000293-128.json"},
        {"os1_128_legacy_fw2_0_windowed_2048",
         "metadata/2_0_0_os1-992008000494-128_col_win_legacy.json"},
        {"os1_128_fusa_dual",
         "pcaps/OS-1-128_767798045_1024x10_20230712_120049.json"},
        {"os0_32_legacy",
         "metadata/1_14_6cccd_os-882002000138-32U0_legacy.json"},
        {"os0_32_modern",
         "pcaps/OS-0-32-U1_v2.2.0_1024x10.json"},
        {"os0_128_legacy_fw1_14",
         "metadata/1_14_6cccd_os-882002000138-128_legacy.json"},
        {"os0_128_fw2_3",
         "pcaps/OS-0-128-U1_v2.3.0_1024x10.json"},
        {"os0_128_fw3_0",
         "pcaps/OS-0-128_v3.0.1_1024x10.json"},
        {"os0_128_fusa_fw3_1", "pcaps/same_ports.1.json"},
        {"os0_128_fw3_2_512", "pcaps/crc_test.json"},
        {"os2_32_fw2_0", "pcaps/OS-2-32-U0_v2.0.0_1024x10.json"},
        {"os2_128_fw2_3", "pcaps/OS-2-128-U1_v2.3.0_1024x10.json"},
    };

    for (const auto& geometry_case : cases) {
        SCOPED_TRACE(geometry_case.name);
        const auto info = load_sdk_test_info(geometry_case.metadata);

        PinholeProcessor::PanelConfig config;
        config.name = geometry_case.name;
        config.width = 256;
        config.height = 0;
        config.crop_to_valid_region = true;

        PinholeProcessor::OutputType output;
        auto process = PinholeProcessor::create(
            info, {config}, "{name}", "", 0.0,
            [&output](PinholeProcessor::OutputType& panels) {
                output = panels;
            });
        ouster::sdk::core::LidarScan scan(info);
        ASSERT_TRUE(
            scan.has_field(ouster::sdk::core::ChanField::RANGE));
        auto range =
            scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
        const std::array<uint32_t, 3> test_ranges_mm{1000, 10000, 100000};
        for (Eigen::Index source_row = 0; source_row < range.rows();
             ++source_row) {
            for (Eigen::Index source_column = 0;
                 source_column < range.cols(); ++source_column) {
                const size_t source_index =
                    static_cast<size_t>(source_row) * range.cols() +
                    static_cast<size_t>(source_column);
                range(source_row, source_column) = source_index % 97 == 0
                    ? 0
                    : test_ranges_mm[source_index % test_ranges_mm.size()];
            }
        }
        process(scan, 0, rclcpp::Time{int64_t{1}, RCL_ROS_TIME});

        ASSERT_EQ(output.size(), 1u);
        const auto& panel = *output.front();
        const auto depth_it =
            panel.depth_images.find(ouster::sdk::core::ChanField::RANGE);
        ASSERT_NE(depth_it, panel.depth_images.end());
        ASSERT_TRUE(depth_it->second);
        const auto& depth_image = *depth_it->second;
        const auto xyz_lut = ouster::sdk::core::make_xyz_lut(
            info.format.columns_per_frame, info.format.pixels_per_column,
            ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
            ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
            info.beam_altitude_angles);
        const auto lut_direction = xyz_lut.direction.cast<float>();
        const auto lut_offset = xyz_lut.offset.cast<float>();
        const Eigen::Matrix3d optical_to_lidar =
            optical_to_lidar_rotation(config.yaw_rad, config.pitch_rad);

        size_t checked = 0;
        size_t zero_range_pixels = 0;
        size_t observably_displaced = 0;
        double squared_position_error_sum = 0.0;
        double max_position_error = 0.0;
        double max_relative_position_error = 0.0;
        double max_optical_depth_error = 0.0;
        double max_reprojection_error = 0.0;
        std::unordered_set<size_t> unique_sources;
        for (uint32_t row = 0; row < depth_image.height; ++row) {
            for (uint32_t column = 0; column < depth_image.width; ++column) {
                const int32_t source_row = panel.r_src(row, column);
                const int32_t source_column =
                    panel.raw_v_src(row, column);
                if (source_row < 0 || source_column < 0) {
                    EXPECT_TRUE(std::isnan(
                        depth_at(depth_image, row, column)));
                    continue;
                }
                const size_t source_index =
                    static_cast<size_t>(source_row) *
                        info.format.columns_per_frame +
                    static_cast<size_t>(source_column);
                const uint32_t raw_range =
                    range(source_row, source_column);
                if (raw_range == 0) {
                    EXPECT_TRUE(std::isnan(
                        depth_at(depth_image, row, column)));
                    ++zero_range_pixels;
                    continue;
                }

                unique_sources.insert(source_index);
                const Eigen::Vector3f raw_lidar_float =
                    (static_cast<float>(raw_range) *
                         lut_direction.row(source_index) +
                     lut_offset.row(source_index))
                        .matrix()
                        .transpose();
                const Eigen::Vector3d raw_lidar =
                    raw_lidar_float.cast<double>();
                ASSERT_TRUE(raw_lidar.allFinite());
                const Eigen::Vector3d raw_optical =
                    optical_to_lidar.transpose() * raw_lidar;
                ASSERT_GT(raw_optical.z(), 0.0);
                const Eigen::Vector3d reconstructed_optical =
                    back_project_depth_pixel(panel, depth_image, row, column);
                ASSERT_TRUE(reconstructed_optical.allFinite());
                const Eigen::Vector3d reconstructed_lidar =
                    optical_to_lidar * reconstructed_optical;

                const double full_column = static_cast<double>(
                    column + panel.camera_info.roi.x_offset);
                const double full_row = static_cast<double>(
                    row + panel.camera_info.roi.y_offset);
                const double projected_column =
                    panel.camera_info.p[0] * raw_optical.x() /
                        raw_optical.z() +
                    panel.camera_info.p[2];
                const double projected_row =
                    panel.camera_info.p[5] * raw_optical.y() /
                        raw_optical.z() +
                    panel.camera_info.p[6];
                const double position_error =
                    (reconstructed_lidar - raw_lidar).norm();
                const double relative_position_error =
                    position_error / raw_lidar.norm();

                max_position_error =
                    std::max(max_position_error, position_error);
                max_relative_position_error = std::max(
                    max_relative_position_error, relative_position_error);
                max_optical_depth_error = std::max(
                    max_optical_depth_error,
                    std::abs(reconstructed_optical.z() - raw_optical.z()));
                max_reprojection_error = std::max(
                    max_reprojection_error,
                    std::hypot(projected_column - full_column,
                               projected_row - full_row));
                squared_position_error_sum +=
                    position_error * position_error;
                if (position_error > 1e-3) ++observably_displaced;
                ++checked;
            }
        }

        ASSERT_GT(checked, 100u);
        EXPECT_GT(zero_range_pixels, 0u);
        EXPECT_LT(max_optical_depth_error, 1e-4);
        EXPECT_GT(observably_displaced, checked / 2);
        // The half-beam sampling error is sensor dependent but remains a
        // small angular error; it therefore scales approximately with range.
        EXPECT_LT(max_relative_position_error, 0.1);
        EXPECT_FALSE(unique_sources.empty());

        const std::string prefix =
            std::string("round_trip_") + geometry_case.name;
        testing::Test::RecordProperty(prefix + "_max_error_m",
                                      max_position_error);
        testing::Test::RecordProperty(
            prefix + "_rms_error_m",
            std::sqrt(squared_position_error_sum /
                      static_cast<double>(checked)));
        testing::Test::RecordProperty(prefix + "_max_relative_error",
                                      max_relative_position_error);
        testing::Test::RecordProperty(prefix + "_max_reprojection_px",
                                      max_reprojection_error);
        testing::Test::RecordProperty(
            prefix + "_unique_source_ratio",
            static_cast<double>(unique_sources.size()) /
                static_cast<double>(checked));
    }
}

TEST(PinholeProcessorTest,
     HandlesSyntheticRev8MaximumResolutionAndBeamCount) {
    const auto info = make_synthetic_rev8_max_info();
    ASSERT_EQ(info.format.pixels_per_column, 256u);
    ASSERT_EQ(info.format.columns_per_frame, 4096u);
    ASSERT_EQ(info.num_returns(), 2);

    PinholeProcessor::PanelConfig config;
    config.name = "rev8_oblique";
    config.yaw_rad = 0.37;
    config.pitch_rad = 0.11;
    config.hfov_rad = 100.0 * M_PI / 180.0;
    config.vfov_rad = 50.0 * M_PI / 180.0;
    config.width = 257;
    config.height = 129;
    config.crop_to_valid_region = false;

    PinholeProcessor::OutputType output;
    auto process = PinholeProcessor::create(
        info, {config}, "{name}", "", 0.0,
        [&output](PinholeProcessor::OutputType& panels) {
            output = panels;
        });
    ouster::sdk::core::LidarScan scan(info);
    auto range =
        scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE);
    auto range2 =
        scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2);
    const std::array<uint32_t, 3> test_ranges_mm{1000, 10000, 100000};
    for (Eigen::Index source_row = 0; source_row < range.rows();
         ++source_row) {
        for (Eigen::Index source_column = 0; source_column < range.cols();
             ++source_column) {
            const size_t source_index =
                static_cast<size_t>(source_row) * range.cols() +
                static_cast<size_t>(source_column);
            const uint32_t value = source_index % 211 == 0
                ? 0
                : test_ranges_mm[source_index % test_ranges_mm.size()];
            range(source_row, source_column) = value;
            range2(source_row, source_column) = value == 0 ? 0 : value * 2;
        }
    }
    process(scan, 0, rclcpp::Time{int64_t{1}, RCL_ROS_TIME});

    ASSERT_EQ(output.size(), 1u);
    const auto& panel = *output.front();
    EXPECT_EQ(panel.images.count(ouster::sdk::core::ChanField::RGB), 0u);
    EXPECT_EQ(PinholeProcessor::channel_topics(info).count(
                  ouster::sdk::core::ChanField::RGB),
              0u);
    EXPECT_EQ(PinholeProcessor::channel_topics(info, true).count(
                  ouster::sdk::core::ChanField::RGB),
              1u);
    ASSERT_EQ(panel.depth_images.size(), 2u);
    const auto& depth = *panel.depth_images.at(
        ouster::sdk::core::ChanField::RANGE);

    const auto xyz_lut = ouster::sdk::core::make_xyz_lut(
        info.format.columns_per_frame, info.format.pixels_per_column,
        ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
        ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
        info.beam_altitude_angles);
    const Eigen::Matrix3d optical_to_lidar =
        optical_to_lidar_rotation(config.yaw_rad, config.pitch_rad);
    const int32_t lidar_width =
        static_cast<int32_t>(info.format.columns_per_frame);

    size_t checked = 0;
    size_t zero_range_pixels = 0;
    double max_relative_position_error = 0.0;
    double max_optical_depth_error = 0.0;
    double max_reprojection_error = 0.0;
    for (uint32_t row = 0; row < depth.height; ++row) {
        for (uint32_t column = 0; column < depth.width; ++column) {
            const int32_t source_row = panel.r_src(row, column);
            const int32_t raw_column = panel.raw_v_src(row, column);
            if (source_row < 0 || raw_column < 0) {
                EXPECT_TRUE(std::isnan(depth_at(depth, row, column)));
                continue;
            }

            int32_t expected_destaggered =
                (raw_column +
                 info.format.pixel_shift_by_row[source_row]) %
                lidar_width;
            if (expected_destaggered < 0) {
                expected_destaggered += lidar_width;
            }
            EXPECT_EQ(panel.v_src(row, column), expected_destaggered);

            const uint32_t raw_range = range(source_row, raw_column);
            if (raw_range == 0) {
                EXPECT_TRUE(std::isnan(depth_at(depth, row, column)));
                ++zero_range_pixels;
                continue;
            }

            const Eigen::Index source_index =
                static_cast<Eigen::Index>(source_row) * lidar_width +
                raw_column;
            const Eigen::Vector3d raw_lidar =
                (static_cast<double>(raw_range) *
                     xyz_lut.direction.row(source_index) +
                 xyz_lut.offset.row(source_index))
                    .matrix()
                    .transpose();
            const Eigen::Vector3d raw_optical =
                optical_to_lidar.transpose() * raw_lidar;
            ASSERT_GT(raw_optical.z(), 0.0);
            const Eigen::Vector3d reconstructed_optical =
                back_project_depth_pixel(panel, depth, row, column);
            const Eigen::Vector3d reconstructed_lidar =
                optical_to_lidar * reconstructed_optical;
            const double projected_column =
                panel.camera_info.p[0] * raw_optical.x() /
                    raw_optical.z() +
                panel.camera_info.p[2];
            const double projected_row =
                panel.camera_info.p[5] * raw_optical.y() /
                    raw_optical.z() +
                panel.camera_info.p[6];

            max_relative_position_error = std::max(
                max_relative_position_error,
                (reconstructed_lidar - raw_lidar).norm() /
                    raw_lidar.norm());
            max_optical_depth_error = std::max(
                max_optical_depth_error,
                std::abs(reconstructed_optical.z() - raw_optical.z()));
            max_reprojection_error = std::max(
                max_reprojection_error,
                std::hypot(projected_column - column,
                           projected_row - row));
            ++checked;
        }
    }

    EXPECT_GT(checked, 20000u);
    EXPECT_GT(zero_range_pixels, 0u);
    EXPECT_LT(max_optical_depth_error, 1e-4);
    EXPECT_LT(max_relative_position_error, 0.05);
    EXPECT_LT(max_reprojection_error, 4.0);
    testing::Test::RecordProperty("rev8_max_relative_error",
                                  max_relative_position_error);
    testing::Test::RecordProperty("rev8_max_reprojection_px",
                                  max_reprojection_error);
}

TEST(PinholeProcessorTest,
     DomeFivePanelPresetCoversTheCalibratedHemisphere) {
    const auto configs = dome_panel_configs(256, true);
    ASSERT_EQ(configs.size(), 5u);
    EXPECT_EQ(configs.back().name, "top");
    EXPECT_NEAR(configs.back().pitch_rad, M_PI_2, 1e-12);

    // Sample the full elevation envelope represented by published OSDome
    // calibration. Four overlapping side panels cover the horizon and the
    // top-facing panel covers the otherwise singular zenith region.
    for (int elevation_tenths = -10; elevation_tenths <= 870;
         ++elevation_tenths) {
        const double elevation_rad =
            static_cast<double>(elevation_tenths) * M_PI / 1800.0;
        for (int azimuth_deg = 0; azimuth_deg < 360; ++azimuth_deg) {
            const double azimuth_rad =
                static_cast<double>(azimuth_deg) * M_PI / 180.0;
            const Eigen::Vector3d direction{
                std::cos(elevation_rad) * std::cos(azimuth_rad),
                std::cos(elevation_rad) * std::sin(azimuth_rad),
                std::sin(elevation_rad)};

            bool covered = false;
            for (const auto& config : configs) {
                const Eigen::Vector3d optical =
                    optical_to_lidar_rotation(config.yaw_rad,
                                              config.pitch_rad)
                        .transpose() *
                    direction;
                if (optical.z() <= 0.0) continue;
                const double horizontal_limit =
                    std::tan(0.5 * config.hfov_rad);
                const double vertical_limit =
                    std::tan(0.5 * config.vfov_rad);
                if (std::abs(optical.x() / optical.z()) <=
                        horizontal_limit &&
                    std::abs(optical.y() / optical.z()) <=
                        vertical_limit) {
                    covered = true;
                    break;
                }
            }

            if (!covered) {
                ADD_FAILURE() << "uncovered direction: azimuth="
                              << azimuth_deg << " deg, elevation="
                              << static_cast<double>(elevation_tenths) / 10.0
                              << " deg";
                return;
            }
        }
    }
}

TEST(PinholeProcessorTest,
     TopFacingPanelHandlesDomeStyleExtremeCalibration) {
    const auto info = make_synthetic_dome_info();
    const auto configs = dome_panel_configs(129, false);

    PinholeProcessor::OutputType output;
    auto process = PinholeProcessor::create(
        info, configs, "{name}", "", 0.0,
        [&output](PinholeProcessor::OutputType& panels) {
            output = panels;
        });
    ouster::sdk::core::LidarScan scan(info);
    scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE)
        .setConstant(10000);
    if (scan.has_field(ouster::sdk::core::ChanField::RANGE2)) {
        scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2)
            .setConstant(20000);
    }
    process(scan, 0, rclcpp::Time{int64_t{1}, RCL_ROS_TIME});

    ASSERT_EQ(output.size(), 5u);
    const auto& panel = *output.back();
    ASSERT_EQ(panel.name, "top");
    ASSERT_EQ(panel.camera_info.width, 129u);
    ASSERT_EQ(panel.camera_info.height, 129u);
    EXPECT_TRUE(
        optical_to_lidar_rotation(panel.yaw_rad, panel.pitch_rad)
            .col(2)
            .isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

    constexpr uint32_t center = 64;
    const auto depth_it =
        panel.depth_images.find(ouster::sdk::core::ChanField::RANGE);
    ASSERT_NE(depth_it, panel.depth_images.end());
    ASSERT_TRUE(depth_it->second);
    const auto& depth_image = *depth_it->second;
    // This synthetic calibration, like representative OSDome metadata, has
    // its highest beam at +87 degrees. Do not invent a return on the exact
    // +90-degree ray in the small physical zenith blind cap.
    EXPECT_EQ(panel.r_src(center, center), -1);
    EXPECT_TRUE(std::isnan(depth_at(depth_image, center, center)));

    const auto xyz_lut = ouster::sdk::core::make_xyz_lut(
        info.format.columns_per_frame, info.format.pixels_per_column,
        ouster::sdk::core::RANGE_UNIT, info.beam_to_lidar_transform,
        ouster::sdk::core::mat4d::Identity(), info.beam_azimuth_angles,
        info.beam_altitude_angles);
    const auto lut_direction = xyz_lut.direction.cast<float>();
    const auto lut_offset = xyz_lut.offset.cast<float>();
    const Eigen::Matrix3d optical_to_lidar =
        optical_to_lidar_rotation(panel.yaw_rad, panel.pitch_rad);

    size_t checked = 0;
    double max_optical_depth_error = 0.0;
    double max_relative_position_error = 0.0;
    double max_reprojection_error = 0.0;
    double nearest_valid_radius = std::numeric_limits<double>::infinity();
    int32_t nearest_valid_source_row = -1;
    for (uint32_t row = 0; row < depth_image.height; ++row) {
        for (uint32_t column = 0; column < depth_image.width; ++column) {
            const int32_t source_row = panel.r_src(row, column);
            const int32_t raw_column = panel.raw_v_src(row, column);
            if (source_row < 0 || raw_column < 0) {
                EXPECT_TRUE(std::isnan(
                    depth_at(depth_image, row, column)));
                continue;
            }

            const double center_radius = std::hypot(
                static_cast<double>(row) - center,
                static_cast<double>(column) - center);
            if (center_radius < nearest_valid_radius) {
                nearest_valid_radius = center_radius;
                nearest_valid_source_row = source_row;
            }

            const int32_t width =
                static_cast<int32_t>(info.format.columns_per_frame);
            int32_t expected_destaggered_column =
                (raw_column +
                 info.format.pixel_shift_by_row[source_row]) %
                width;
            if (expected_destaggered_column < 0) {
                expected_destaggered_column += width;
            }
            EXPECT_EQ(panel.v_src(row, column),
                      expected_destaggered_column);

            const size_t source_index =
                static_cast<size_t>(source_row) * width + raw_column;
            const Eigen::Vector3d raw_lidar =
                (10000.0f * lut_direction.row(source_index) +
                 lut_offset.row(source_index))
                    .matrix()
                    .transpose()
                    .cast<double>();
            const Eigen::Vector3d raw_optical =
                optical_to_lidar.transpose() * raw_lidar;
            ASSERT_GT(raw_optical.z(), 0.0);
            const Eigen::Vector3d reconstructed_optical =
                back_project_depth_pixel(panel, depth_image, row, column);
            const Eigen::Vector3d reconstructed_lidar =
                optical_to_lidar * reconstructed_optical;

            const double projected_column =
                panel.camera_info.p[0] * raw_optical.x() /
                    raw_optical.z() +
                panel.camera_info.p[2];
            const double projected_row =
                panel.camera_info.p[5] * raw_optical.y() /
                    raw_optical.z() +
                panel.camera_info.p[6];
            max_optical_depth_error = std::max(
                max_optical_depth_error,
                std::abs(reconstructed_optical.z() - raw_optical.z()));
            max_relative_position_error = std::max(
                max_relative_position_error,
                (reconstructed_lidar - raw_lidar).norm() /
                    raw_lidar.norm());
            max_reprojection_error = std::max(
                max_reprojection_error,
                std::hypot(projected_column - column,
                           projected_row - row));
            ++checked;
        }
    }

    EXPECT_GT(checked, static_cast<size_t>(16000));
    EXPECT_LE(nearest_valid_radius, 3.0);
    ASSERT_GE(nearest_valid_source_row, 0);
    EXPECT_GT(info.beam_altitude_angles[nearest_valid_source_row], 86.0);
    EXPECT_LT(max_optical_depth_error, 1e-4);
    EXPECT_LT(max_relative_position_error, 0.08);
    EXPECT_LT(max_reprojection_error, 4.0);
    testing::Test::RecordProperty("dome_max_relative_error",
                                  max_relative_position_error);
    testing::Test::RecordProperty("dome_max_reprojection_px",
                                  max_reprojection_error);
}

TEST(PinholeProcessorTest, AccountsForEverySdkLidarProfile) {
    using Profile = ouster::sdk::core::UDPProfileLidar;
    // Keep this list exhaustive for the SDK pinned by ouster-ros. UNKNOWN is
    // invalid, OFF intentionally has no lidar pixels, and FIVE_WORD_PIXEL is
    // the SDK's raw-word debug profile. Every production range profile must
    // reach the common pinhole path below.
    const std::array<Profile, 14> profiles{
        Profile::UNKNOWN,
        Profile::LEGACY,
        Profile::RNG19_RFL8_SIG16_NIR16_DUAL,
        Profile::RNG19_RFL8_SIG16_NIR16,
        Profile::RNG15_RFL8_NIR8,
        Profile::FIVE_WORD_PIXEL,
        Profile::FUSA_RNG15_RFL8_NIR8_DUAL,
        Profile::RNG15_RFL8_NIR8_DUAL,
        Profile::RNG15_RFL8_NIR8_ZONE16,
        Profile::RNG19_RFL8_SIG16_NIR16_ZONE16,
        Profile::RNG15_RFL8_WIN8,
        Profile::RNG19_RFL8_SIG16_NIR16_RGB16,
        Profile::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL,
        Profile::OFF,
    };

    size_t production_profiles_processed = 0;
    for (const auto profile : profiles) {
        SCOPED_TRACE(ouster::sdk::core::to_string(profile));
        auto info = load_test_info();
        info.format.udp_profile_lidar = profile;
        if (profile == Profile::UNKNOWN) {
            EXPECT_THROW(
                {
                    const ouster::sdk::core::LidarScan unknown_scan{info};
                    (void)unknown_scan;
                },
                std::invalid_argument);
            continue;
        }

        ouster::sdk::core::LidarScan scan(info);
        if (profile == Profile::FIVE_WORD_PIXEL) {
            EXPECT_FALSE(scan.has_field(ouster::sdk::core::ChanField::RANGE));
            EXPECT_TRUE(
                scan.has_field(ouster::sdk::core::ChanField::RAW32_WORD1));
            EXPECT_TRUE(
                scan.has_field(ouster::sdk::core::ChanField::RAW32_WORD5));
            continue;
        }
        if (profile == Profile::OFF) {
            EXPECT_FALSE(scan.has_field(ouster::sdk::core::ChanField::RANGE));
            continue;
        }

        ++production_profiles_processed;
        ASSERT_TRUE(scan.has_field(ouster::sdk::core::ChanField::RANGE));
        scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE)
            .setConstant(10000);
        if (info.num_returns() == 2) {
            ASSERT_TRUE(
                scan.has_field(ouster::sdk::core::ChanField::RANGE2));
            scan.field<uint32_t>(ouster::sdk::core::ChanField::RANGE2)
                .setConstant(20000);
        }

        PinholeProcessor::PanelConfig config;
        config.name = "front";
        config.width = 9;
        config.height = 5;
        config.crop_to_valid_region = false;
        PinholeProcessor::OutputType output;
        auto process = PinholeProcessor::create(
            info, {config}, "{name}", "", 0.0,
            [&output](PinholeProcessor::OutputType& panels) {
                output = panels;
            });
        ASSERT_NO_THROW(process(
            scan, 0, rclcpp::Time{int64_t{1}, RCL_ROS_TIME}));
        ASSERT_EQ(output.size(), 1u);
        ASSERT_EQ(output.front()->depth_images.size(),
                  static_cast<size_t>(info.num_returns()));
        EXPECT_TRUE(std::isfinite(
            depth_at(*output.front()
                          ->depth_images.at(
                              ouster::sdk::core::ChanField::RANGE),
                     2, 4)));
        if (info.num_returns() == 2) {
            EXPECT_TRUE(std::isfinite(
                depth_at(*output.front()
                              ->depth_images.at(
                                  ouster::sdk::core::ChanField::RANGE2),
                         2, 4)));
        }
    }
    EXPECT_EQ(production_profiles_processed, 11u);
}

TEST(PinholeProcessorTest, RgbPanelPublicationIsExplicitAndProfileGated) {
    using ouster::sdk::core::UDPProfileLidar;
    using ouster::sdk::core::fd_array;

    const auto non_rgb_info = load_test_info();
    EXPECT_EQ(PinholeProcessor::channel_topics(non_rgb_info, true).count(
                  ChanField::RGB),
              0u);

    const std::array<UDPProfileLidar, 2> rgb_profiles{
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16,
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_RGB16_DUAL};
    for (const auto profile : rgb_profiles) {
        SCOPED_TRACE(ouster::sdk::core::to_string(profile));
        auto info = load_test_info();
        info.format.udp_profile_lidar = profile;
        EXPECT_EQ(PinholeProcessor::channel_topics(info).count(
                      ChanField::RGB),
                  0u);
        EXPECT_EQ(PinholeProcessor::channel_topics(info, true).at(
                      ChanField::RGB),
                  "rgb_image");

        PinholeProcessor::PanelConfig config;
        config.name = "front";
        config.width = 17;
        config.height = 9;
        config.crop_to_valid_region = false;

        PinholeProcessor::OutputType output;
        auto process = PinholeProcessor::create(
            info, {config}, "{name}", "", 0.0,
            [&output](PinholeProcessor::OutputType& panels) {
                output = panels;
            },
            true);

        ouster::sdk::core::LidarScan scan(info);
        ASSERT_FALSE(scan.has_field(ChanField::R_U8));
        scan.add_field(ChanField::R_U8,
                       fd_array<uint8_t>(info.format.pixels_per_column,
                                         info.format.columns_per_frame));
        scan.add_field(ChanField::G_U8,
                       fd_array<uint8_t>(info.format.pixels_per_column,
                                         info.format.columns_per_frame));
        scan.add_field(ChanField::B_U8,
                       fd_array<uint8_t>(info.format.pixels_per_column,
                                         info.format.columns_per_frame));
        auto red = scan.field<uint8_t>(ChanField::R_U8);
        auto green = scan.field<uint8_t>(ChanField::G_U8);
        auto blue = scan.field<uint8_t>(ChanField::B_U8);
        for (Eigen::Index row = 0; row < red.rows(); ++row) {
            for (Eigen::Index column = 0; column < red.cols(); ++column) {
                red(row, column) =
                    static_cast<uint8_t>(1 + row % 250);
                green(row, column) =
                    static_cast<uint8_t>(1 + column % 250);
                blue(row, column) = static_cast<uint8_t>(
                    1 + (row + column) % 250);
            }
        }
        scan.field<uint32_t>(ChanField::RANGE).setConstant(10000);
        if (info.num_returns() == 2) {
            scan.field<uint32_t>(ChanField::RANGE2).setConstant(20000);
        }

        const rclcpp::Time stamp{int64_t{123456789}, RCL_ROS_TIME};
        process(scan, 0, stamp);
        ASSERT_EQ(output.size(), 1u);
        const auto& panel = *output.front();
        const auto rgb_it = panel.images.find(ChanField::RGB);
        ASSERT_NE(rgb_it, panel.images.end());
        ASSERT_TRUE(rgb_it->second);
        const auto& rgb = *rgb_it->second;
        EXPECT_EQ(rgb.encoding, sensor_msgs::image_encodings::RGB8);
        EXPECT_EQ(rgb.step, rgb.width * 3u);
        EXPECT_EQ(rgb.data.size(),
                  static_cast<size_t>(rgb.step) * rgb.height);
        EXPECT_EQ(rclcpp::Time(rgb.header.stamp), stamp);

        size_t valid_pixels = 0;
        size_t invalid_pixels = 0;
        for (uint32_t row = 0; row < rgb.height; ++row) {
            for (uint32_t column = 0; column < rgb.width; ++column) {
                const int32_t source_row = panel.r_src(row, column);
                const int32_t raw_column =
                    panel.raw_v_src(row, column);
                if (source_row < 0 || raw_column < 0) {
                    EXPECT_EQ(rgb8_at(rgb, row, column),
                              (std::array<uint8_t, 3>{0, 0, 0}));
                    ++invalid_pixels;
                    continue;
                }
                EXPECT_EQ(
                    rgb8_at(rgb, row, column),
                    (std::array<uint8_t, 3>{
                        red(source_row, raw_column),
                        green(source_row, raw_column),
                        blue(source_row, raw_column)}));
                ++valid_pixels;
            }
        }
        EXPECT_GT(valid_pixels, 0u);
        EXPECT_GT(invalid_pixels, 0u);
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
    const int32_t raw_column = panel.raw_v_src(row, column);
    ASSERT_GE(raw_column, 0);

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

TEST(PinholeProcessorTest, SupportsBundledFirmwareMetadataSchemas) {
    const std::vector<std::string> filenames{
        "1_12_os1-991913000010-64.json",
        "1_12_os1-991937000062-64_legacy.json",
        "1_13_os1-991913000010-64.json",
        "1_14_6cccd_os-882002000138-128_legacy.json",
        "1_14_6cccd_os-882002000138-32U0_legacy.json",
        "1_14_beta_os1-991937000062-16A0_legacy.json",
        "1_14_beta_os1-991937000062-64_legacy.json",
        "2_0_0_os1-991913000010-64.json",
        "2_0_0_os1-992008000494-128_col_win_legacy.json",
        "2_0_rc2_os-992011000121-32U0_legacy.json",
        "2_1_2_os1-991913000010-64.json",
        "2_1_2_os1-991913000010-64_legacy.json",
        "2_2_os-992119000444-128.json",
        "2_2_os-992119000444-128_legacy.json",
        "2_3_1_os-992146000760-128.json",
        "2_3_1_os-992146000760-128_legacy.json",
        "2_4_0_os-992146000760-128.json",
        "2_4_0_os-992146000760-128_legacy.json",
        "2_5_0_os-992146000760-128.json",
        "2_5_0_os-992146000760-128_legacy.json",
        "3_0_1_os-122246000293-128.json",
        "3_0_1_os-122246000293-128_legacy.json",
    };

    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;
    config.crop_to_valid_region = false;

    for (const auto& filename : filenames) {
        SCOPED_TRACE(filename);
        const auto info = load_test_info(filename);
        EXPECT_NO_THROW(PinholeProcessor(info, {config}, "{name}", "",
                                         0.0, nullptr));
    }
}

TEST(PinholeProcessorTest,
     DocumentsPre114ReducedChannelMetadataRejectedBySdk) {
    // These historical SDK fixtures identify themselves as 16/32-channel
    // products but contain the original 64 calibration entries. The current
    // SensorInfo parser rejects them before any ROS image processor can run.
    // Keep this compatibility boundary explicit instead of accidentally
    // claiming that the pinhole path supports metadata the SDK cannot parse.
    const std::vector<std::string> filenames{
        "1_12_os1-991937000062-16A0_legacy.json",
        "1_13_os1-991937000062-16A0_legacy.json",
        "1_13_os1-991937000062-32A02_legacy.json",
    };

    for (const auto& filename : filenames) {
        SCOPED_TRACE(filename);
        EXPECT_ANY_THROW(load_test_info(filename));
    }
}

TEST(PinholeProcessorTest,
     DocumentsFirmware32ImuOnlyMetadataHasNoLidarFields) {
    // This fixture carries valid OS0 calibration and lidar_data_format while
    // recording only IMU and zone-monitoring traffic with lidar disabled. The
    // SDK therefore creates an intentionally fieldless LidarScan; it is useful
    // metadata-schema coverage, but not evidence of a working lidar profile.
    const auto info =
        load_sdk_test_info("pcaps/imu_zm_no_lidar_0.json");

    PinholeProcessor::PanelConfig config;
    config.name = "front";
    config.width = 4;
    config.height = 2;
    config.crop_to_valid_region = false;
    EXPECT_NO_THROW(PinholeProcessor(info, {config}, "{name}", "", 0.0,
                                     nullptr));

    const ouster::sdk::core::LidarScan scan(info);
    EXPECT_FALSE(
        scan.has_field(ouster::sdk::core::ChanField::RANGE));
}

}  // namespace
}  // namespace ouster_ros
