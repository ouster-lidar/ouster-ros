// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "ouster_ros/sensor_point_types.h"
#include "ouster_ros/common_point_types.h"
#include "ouster_ros/os_point.h"
#include "../src/point_meta_helpers.h"
#include "../src/point_cloud_compose.h"
#include "../src/point_cloud_processor.h"

class PointCloudComposeTest : public ::testing::Test {
   protected:
    void SetUp() override {}

    void TearDown() override {}
};

using namespace std;
using namespace ouster_ros;
using namespace ouster::sdk::core;

// TODO: generalize the test case!

TEST_F(PointCloudComposeTest, MapLidarScanFields) {
    const auto WIDTH = 5U;
    const auto HEIGHT = 3U;
    const auto SAMPLES = WIDTH * HEIGHT;
    UDPProfileLidar lidar_udp_profile =
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;

    LidarScan ls(WIDTH, HEIGHT, lidar_udp_profile);

    auto fill_data = [](auto& img, auto base, auto count) {
        auto* p = img.data();
        for (auto i = 0U; i < count; ++i)
            p[i] =
                static_cast<std::remove_reference_t<decltype(p[0])>>(base + i);
    };

    auto range = ls.field<uint32_t>(ChanField::RANGE);
    auto signal = ls.field<uint16_t>(ChanField::SIGNAL);
    auto reflect = ls.field<uint8_t>(ChanField::REFLECTIVITY);
    auto near_ir = ls.field<uint16_t>(ChanField::NEAR_IR);

    // choose a base value that could ultimately wrap around
    fill_data(range, static_cast<uint32_t>(1 + (1ULL << 32) - SAMPLES / 2),
              SAMPLES);
    fill_data(signal, static_cast<uint16_t>(3 + (1 << 16) - SAMPLES / 2),
              SAMPLES);
    fill_data(reflect, static_cast<uint8_t>(5 + (1 << 8) - SAMPLES / 2),
              SAMPLES);
    fill_data(near_ir, static_cast<uint16_t>(7 + (1 << 16) - SAMPLES / 2),
              SAMPLES);

    ouster_ros::Cloud<Point_RNG19_RFL8_SIG16_NIR16_DUAL> cloud{WIDTH, HEIGHT};

    auto ls_tuple =
        make_lidar_scan_tuple<0, Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
                              Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(ls);

    ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_DUAL pt;

    for (auto src_idx = 0U; src_idx < SAMPLES; ++src_idx) {
        copy_lidar_scan_fields_to_point<0>(pt, ls_tuple, src_idx);
        EXPECT_EQ(point::get<5>(pt), range.data()[src_idx]);
        EXPECT_EQ(point::get<6>(pt), signal.data()[src_idx]);
        EXPECT_EQ(point::get<7>(pt), reflect.data()[src_idx]);
        EXPECT_EQ(point::get<8>(pt), near_ir.data()[src_idx]);
    }
}

TEST_F(PointCloudComposeTest, ScanToCloudDestaggerHandlesNegativePixelShift) {
    const auto WIDTH = 8;
    const auto HEIGHT = 6;
    const auto SAMPLES = WIDTH * HEIGHT;

    LidarScan ls(WIDTH, HEIGHT, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    auto range = ls.field<uint32_t>(ChanField::RANGE);
    for (auto i = 0; i < SAMPLES; ++i) range.data()[i] = i + 1;

    PointCloudXYZf points = PointCloudXYZf::Zero(SAMPLES, 3);

    // mix of positive, zero, negative and boundary shifts within [-w, w)
    const std::vector<int> pixel_shift_by_row{3, 0, -3, WIDTH - 1, -WIDTH, -1};

    ouster_ros::Cloud<Point_RNG19_RFL8_SIG16_NIR16> cloud{WIDTH, HEIGHT};
    Point_RNG19_RFL8_SIG16_NIR16 staging_pt;
    scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16.size(),
                    Profile_RNG19_RFL8_SIG16_NIR16>(
        cloud, staging_pt, points, 0, ls, pixel_shift_by_row, true, true);

    for (auto u = 0; u < HEIGHT; ++u) {
        const auto s = ((pixel_shift_by_row[u] % WIDTH) + WIDTH) % WIDTH;
        for (auto v = 0; v < WIDTH; ++v) {
            const auto expected_v = (v + WIDTH - s) % WIDTH;
            EXPECT_EQ(cloud.points[u * WIDTH + v].range,
                      range(u, expected_v))
                << "u=" << u << " v=" << v;
        }
    }
}

namespace {
PointCloudProcessor<Point_RNG19_RFL8_SIG16_NIR16> make_processor(
    const SensorInfo& info) {
    return PointCloudProcessor<Point_RNG19_RFL8_SIG16_NIR16>(
        info, "os_lidar", false, 0, 1000000, 1, "",
        [](auto&, const auto&, auto, const auto&, const auto&, auto) {},
        [](auto) {});
}
}  // namespace

TEST_F(PointCloudComposeTest, PointCloudProcessorAcceptsPixelShiftInRange) {
    auto info = default_sensor_info(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);
    shifts.front() = -w;
    shifts.back() = w - 1;
    EXPECT_NO_THROW(make_processor(info));
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftOutOfRange) {
    auto info = default_sensor_info(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);

    shifts[1] = w;
    EXPECT_THROW(make_processor(info), std::invalid_argument);

    shifts[1] = -w - 1;
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftSizeMismatch) {
    auto info = default_sensor_info(LidarMode::_512x10);
    info.format.pixel_shift_by_row.pop_back();
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}
