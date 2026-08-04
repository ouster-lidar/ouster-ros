#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>

#include <limits>
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
        EXPECT_EQ(point::get<5>(pt), range(0, src_idx));
        EXPECT_EQ(point::get<6>(pt), signal(0, src_idx));
        EXPECT_EQ(point::get<7>(pt), reflect(0, src_idx));
        EXPECT_EQ(point::get<8>(pt), near_ir(0, src_idx));
    }
}

TEST_F(PointCloudComposeTest, DestaggerNormalizesPositiveAndNegativeShifts) {
    constexpr uint32_t WIDTH = 5;
    constexpr uint32_t HEIGHT = 1;
    LidarScan ls(WIDTH, HEIGHT,
                 UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);

    PointCloudXYZf points(WIDTH * HEIGHT, 3);
    points.setZero();
    for (uint32_t i = 0; i < WIDTH; ++i) points(i, 0) = i;

    auto compose = [&](int shift) {
        Cloud<Point_RNG19_RFL8_SIG16_NIR16> cloud{WIDTH, HEIGHT};
        Point_RNG19_RFL8_SIG16_NIR16 staging_point;
        scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16.size(),
                        Profile_RNG19_RFL8_SIG16_NIR16>(
            cloud, staging_point, points, 0, ls, {shift},
            /*organized=*/true, /*destagger=*/true);
        std::vector<float> x;
        for (const auto& point : cloud.points) x.push_back(point.x);
        return x;
    };

    EXPECT_EQ(compose(2), (std::vector<float>{3, 4, 0, 1, 2}));
    EXPECT_EQ(compose(-1), (std::vector<float>{1, 2, 3, 4, 0}));
    EXPECT_EQ(compose(7), (std::vector<float>{3, 4, 0, 1, 2}));
}

TEST_F(PointCloudComposeTest, UnorganizedDestaggerAdvancesPastInvalidPoints) {
    constexpr uint32_t WIDTH = 5;
    constexpr uint32_t HEIGHT = 1;
    LidarScan ls(WIDTH, HEIGHT,
                 UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);

    PointCloudXYZf points(WIDTH * HEIGHT, 3);
    points.setZero();
    for (uint32_t i = 0; i < WIDTH; ++i) points(i, 0) = i;
    points(3, 0) = std::numeric_limits<float>::quiet_NaN();

    Cloud<Point_RNG19_RFL8_SIG16_NIR16> cloud;
    Point_RNG19_RFL8_SIG16_NIR16 staging_point;
    scan_to_cloud_f<Profile_RNG19_RFL8_SIG16_NIR16.size(),
                    Profile_RNG19_RFL8_SIG16_NIR16>(
        cloud, staging_point, points, 0, ls, {2},
        /*organized=*/false, /*destagger=*/true);

    ASSERT_EQ(cloud.size(), 4U);
    EXPECT_EQ(cloud.points[0].x, 4);
    EXPECT_EQ(cloud.points[1].x, 0);
    EXPECT_EQ(cloud.points[2].x, 1);
    EXPECT_EQ(cloud.points[3].x, 2);
}
