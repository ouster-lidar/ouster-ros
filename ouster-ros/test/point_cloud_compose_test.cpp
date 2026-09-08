// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>

#include <vector>

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

TEST_F(PointCloudComposeTest, DestaggersSignedAndOutOfRangeRowShifts) {
    constexpr auto WIDTH = 5U;
    constexpr auto HEIGHT = 3U;
    constexpr auto SAMPLES = WIDTH * HEIGHT;

    LidarScan ls(WIDTH, HEIGHT,
                 UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    auto range = ls.field<uint32_t>(ChanField::RANGE);
    PointCloudXYZf points(SAMPLES, 3);
    for (auto src_idx = 0U; src_idx < SAMPLES; ++src_idx) {
        range.data()[src_idx] = 1000U + src_idx;
        points(src_idx, 0) = static_cast<float>(src_idx);
        points(src_idx, 1) = static_cast<float>(src_idx) + 0.25F;
        points(src_idx, 2) = -static_cast<float>(src_idx);
    }

    // Include a conventional positive shift, a signed modern-firmware shift,
    // and an equivalent shift outside a single image width.
    const std::vector<int> pixel_shift_by_row{2, -1, 7};
    Cloud<Point_RNG19_RFL8_SIG16_NIR16_DUAL> cloud{WIDTH, HEIGHT};
    Point_RNG19_RFL8_SIG16_NIR16_DUAL staging_point;
    scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
                    Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(
        cloud, staging_point, points, 0, ls, pixel_shift_by_row, true, true);

    for (auto row = 0U; row < HEIGHT; ++row) {
        const auto normalized_shift =
            (static_cast<int>(WIDTH) +
             pixel_shift_by_row[row] % static_cast<int>(WIDTH)) %
            static_cast<int>(WIDTH);
        for (auto dest_col = 0U; dest_col < WIDTH; ++dest_col) {
            const auto raw_col =
                (dest_col + WIDTH - static_cast<unsigned>(normalized_shift)) %
                WIDTH;
            const auto src_idx = row * WIDTH + raw_col;
            const auto tgt_idx = row * WIDTH + dest_col;
            const auto& point = cloud.points[tgt_idx];

            EXPECT_FLOAT_EQ(point.x, points(src_idx, 0));
            EXPECT_FLOAT_EQ(point.y, points(src_idx, 1));
            EXPECT_FLOAT_EQ(point.z, points(src_idx, 2));
            EXPECT_EQ(point.ring, row);
            EXPECT_EQ(point.range, range.data()[src_idx]);
        }
    }
}
