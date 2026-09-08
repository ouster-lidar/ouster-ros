#include <gtest/gtest.h>
#include <ouster/lidar_scan.h>

#include <algorithm>
#include <cstdint>
#include <type_traits>
#include <utility>

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
#include "../src/point_cloud_processor_factory.h"

namespace {

constexpr auto TEST_WIDTH = 8U;
constexpr auto TEST_HEIGHT = 4U;

template <std::size_t N, const ouster_ros::ChanFieldTable<N>& Profile,
          typename PointT>
void expect_missing_window_is_zero(
    ouster::sdk::core::UDPProfileLidar profile) {
    using namespace ouster::sdk::core;

    static_assert(Profile[0].second == ChanFieldType::UINT32);
    LidarScan scan(TEST_WIDTH, TEST_HEIGHT, profile);
    ASSERT_TRUE(scan.has_field(ChanField::WINDOW));
    if constexpr (std::is_same_v<PointT, ouster_ros::Point_RNG15_RFL8_WIN8>) {
        scan.field<uint8_t>(ChanField::FLAGS)(0, 0) = 42U;
    }
    scan.del_field(ChanField::WINDOW);

    auto first_field = scan.field<uint32_t>(Profile[0].first);
    first_field(0, 0) = 123456U;
    const auto fields =
        ouster_ros::make_lidar_scan_tuple<0, Profile.size(), Profile>(scan);

    PointT point;
    point.window = 0xffU;
    ouster_ros::copy_lidar_scan_fields_to_point<0>(point, fields, 0);

    EXPECT_EQ(ouster_ros::point::get<5>(point), first_field(0, 0));
    EXPECT_EQ(point.window, 0U);
    if constexpr (std::is_same_v<PointT, ouster_ros::Point_RNG15_RFL8_WIN8>) {
        EXPECT_EQ(point.flags, 42U);
    }
}

ouster::sdk::core::SensorInfo make_sensor_info(
    ouster::sdk::core::UDPProfileLidar profile) {
    using namespace ouster::sdk::core;

    auto info = default_sensor_info(LidarMode::_512x10);
    info.image_rev = "v3.1.0";
    info.format.udp_profile_lidar = profile;
    info.format.columns_per_frame = TEST_WIDTH;
    info.format.pixels_per_column = TEST_HEIGHT;
    info.format.columns_per_packet = 4U;
    info.format.column_window = {0, TEST_WIDTH - 1};
    info.format.pixel_shift_by_row.assign(TEST_HEIGHT, 0);
    info.beam_azimuth_angles.assign(TEST_HEIGHT, 0.0);
    info.beam_altitude_angles.assign(TEST_HEIGHT, 0.0);
    return info;
}

void expect_window_value(
    const ouster_ros::PointCloudProcessor_OutputType& messages,
    uint8_t expected) {
    for (const auto& message : messages) {
        const auto window_field = std::find_if(
            message->fields.begin(), message->fields.end(),
            [](const auto& field) { return field.name == "window"; });
        ASSERT_NE(window_field, message->fields.end());
        ASSERT_EQ(window_field->datatype,
                  sensor_msgs::msg::PointField::UINT8);

        for (uint32_t row = 0; row < message->height; ++row) {
            for (uint32_t col = 0; col < message->width; ++col) {
                const auto index = row * message->row_step +
                                   col * message->point_step +
                                   window_field->offset;
                ASSERT_LT(index, message->data.size());
                EXPECT_EQ(message->data[index], expected);
            }
        }
    }
}

void expect_factory_processes_pre32_scan(
    ouster::sdk::core::UDPProfileLidar profile,
    std::size_t expected_returns) {
    using namespace ouster::sdk::core;

    const auto info = make_sensor_info(profile);
    LidarScan scan(info);
    ASSERT_FALSE(scan.has_field(ChanField::WINDOW));
    scan.field<uint32_t>(ChanField::RANGE).setConstant(1000U);
    if (scan.has_field(ChanField::RANGE2)) {
        scan.field<uint32_t>(ChanField::RANGE2).setConstant(2000U);
    }

    ouster_ros::PointCloudProcessor_OutputType output;
    auto processor =
        ouster_ros::PointCloudProcessorFactory::create_point_cloud_processor(
            "native", info, "os_lidar", false, true, false, 0U, 200000U,
            1, "", [&](auto messages) { output = std::move(messages); });

    ASSERT_NO_THROW(processor(scan, 0U, rclcpp::Time(0, 0, RCL_ROS_TIME)));
    ASSERT_EQ(output.size(), expected_returns);
    expect_window_value(output, 0U);
}

}  // namespace

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
    auto window = ls.field<uint8_t>(ChanField::WINDOW);

    // choose a base value that could ultimately wrap around
    fill_data(range, static_cast<uint32_t>(1 + (1ULL << 32) - SAMPLES / 2),
              SAMPLES);
    fill_data(signal, static_cast<uint16_t>(3 + (1 << 16) - SAMPLES / 2),
              SAMPLES);
    fill_data(reflect, static_cast<uint8_t>(5 + (1 << 8) - SAMPLES / 2),
              SAMPLES);
    fill_data(near_ir, static_cast<uint16_t>(7 + (1 << 16) - SAMPLES / 2),
              SAMPLES);
    fill_data(window, static_cast<uint8_t>(11 + (1 << 8) - SAMPLES / 2),
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
        EXPECT_EQ(point::get<10>(pt), window.data()[src_idx]);
    }
}

TEST_F(PointCloudComposeTest, ZeroFillsMissingWindowForEveryNativeLayout) {
    expect_missing_window_is_zero<
        Profile_RNG19_RFL8_SIG16_NIR16.size(),
        Profile_RNG19_RFL8_SIG16_NIR16,
        Point_RNG19_RFL8_SIG16_NIR16>(
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    expect_missing_window_is_zero<
        Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
        Profile_RNG19_RFL8_SIG16_NIR16_DUAL,
        Point_RNG19_RFL8_SIG16_NIR16_DUAL>(
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    expect_missing_window_is_zero<
        Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN.size(),
        Profile_RNG19_RFL8_SIG16_NIR16_DUAL_2ND_RETURN,
        Point_RNG19_RFL8_SIG16_NIR16_DUAL>(
            UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL);
    expect_missing_window_is_zero<
        Profile_RNG15_RFL8_NIR8_DUAL.size(),
        Profile_RNG15_RFL8_NIR8_DUAL,
        Point_RNG15_RFL8_NIR8_DUAL>(
            UDPProfileLidar::RNG15_RFL8_NIR8_DUAL);
    expect_missing_window_is_zero<
        Profile_RNG15_RFL8_NIR8_DUAL_2ND_RETURN.size(),
        Profile_RNG15_RFL8_NIR8_DUAL_2ND_RETURN,
        Point_RNG15_RFL8_NIR8_DUAL>(
            UDPProfileLidar::RNG15_RFL8_NIR8_DUAL);
    expect_missing_window_is_zero<
        Profile_RNG15_RFL8_WIN8.size(),
        Profile_RNG15_RFL8_WIN8,
        Point_RNG15_RFL8_WIN8>(
            UDPProfileLidar::RNG15_RFL8_WIN8);
}

TEST_F(PointCloudComposeTest, MissingRequiredFieldStillThrows) {
    LidarScan scan(TEST_WIDTH, TEST_HEIGHT,
                   UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
    scan.del_field(ChanField::SIGNAL);

    EXPECT_THROW(
        (make_lidar_scan_tuple<
            0, Profile_RNG19_RFL8_SIG16_NIR16.size(),
            Profile_RNG19_RFL8_SIG16_NIR16>(scan)),
        std::out_of_range);
}

TEST_F(PointCloudComposeTest, FactoryProcessesEveryPre32WindowProfile) {
    const std::pair<UDPProfileLidar, std::size_t> profiles[] = {
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, 1U},
        {UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL, 2U},
        {UDPProfileLidar::FUSA_RNG15_RFL8_NIR8_DUAL, 2U},
        {UDPProfileLidar::RNG15_RFL8_NIR8_DUAL, 2U},
        {UDPProfileLidar::RNG15_RFL8_WIN8, 1U},
    };

    for (const auto& [profile, returns] : profiles) {
        SCOPED_TRACE(to_string(profile));
        expect_factory_processes_pre32_scan(profile, returns);
    }
}
