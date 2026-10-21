#include <gtest/gtest.h>
#include <ouster/core/lidar_frame.h>

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
#include "../src/point_cloud_processor_factory.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

class PointCloudComposeTest : public ::testing::Test {
   protected:
    void SetUp() override {}

    void TearDown() override {}
};

using namespace std;
using namespace ouster_ros;
using namespace ouster::sdk::core;

// TODO: generalize the test case!

namespace {
LidarFrame make_lidar_frame(size_t w, size_t h, UDPProfileLidar profile) {
    DataFormat format;
    format.udp_profile_lidar = profile;
    // firmware versions older than 3.2.1 drop the WINDOW field
    return LidarFrame(h, w, get_field_types(format, Version(3, 2, 1)),
                      DEFAULT_COLUMNS_PER_PACKET);
}
}  // namespace

TEST_F(PointCloudComposeTest, MapLidarFrameFields) {
    const auto WIDTH = 5U;
    const auto HEIGHT = 3U;
    const auto SAMPLES = WIDTH * HEIGHT;
    UDPProfileLidar lidar_udp_profile =
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL;

    auto ls = make_lidar_frame(WIDTH, HEIGHT, lidar_udp_profile);

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
        make_lidar_frame_tuple<0, Profile_RNG19_RFL8_SIG16_NIR16_DUAL.size(),
                              Profile_RNG19_RFL8_SIG16_NIR16_DUAL>(ls);

    ouster_ros::Point_RNG19_RFL8_SIG16_NIR16_DUAL pt;

    for (auto src_idx = 0U; src_idx < SAMPLES; ++src_idx) {
        copy_lidar_frame_fields_to_point<0>(pt, ls_tuple, src_idx);
        EXPECT_EQ(point::get<5>(pt), range(0, src_idx));
        EXPECT_EQ(point::get<6>(pt), signal(0, src_idx));
        EXPECT_EQ(point::get<7>(pt), reflect(0, src_idx));
        EXPECT_EQ(point::get<8>(pt), near_ir(0, src_idx));
    }
}

TEST_F(PointCloudComposeTest, ScanToCloudDestaggerHandlesNegativePixelShift) {
    const auto WIDTH = 8;
    const auto HEIGHT = 6;
    const auto SAMPLES = WIDTH * HEIGHT;

    auto ls = make_lidar_frame(WIDTH, HEIGHT, UDPProfileLidar::RNG19_RFL8_SIG16_NIR16);
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
    auto info = *SensorInfo::from_default(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);
    shifts.front() = -w;
    shifts.back() = w - 1;
    EXPECT_NO_THROW(make_processor(info));
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftOutOfRange) {
    auto info = *SensorInfo::from_default(LidarMode::_512x10);
    const int w = static_cast<int>(info.format.columns_per_frame);
    auto& shifts = info.format.pixel_shift_by_row;
    shifts.assign(shifts.size(), 0);

    shifts[1] = w;
    EXPECT_THROW(make_processor(info), std::invalid_argument);

    shifts[1] = -w - 1;
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}

TEST_F(PointCloudComposeTest, PointCloudProcessorRejectsPixelShiftSizeMismatch) {
    auto info = *SensorInfo::from_default(LidarMode::_512x10);
    info.format.pixel_shift_by_row.pop_back();
    EXPECT_THROW(make_processor(info), std::invalid_argument);
}

namespace {
// profiles whose ouster-ros tables carry the WINDOW field
const std::vector<UDPProfileLidar> WINDOW_PROFILES{
    UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL,
    UDPProfileLidar::RNG19_RFL8_SIG16_NIR16,
    UDPProfileLidar::RNG15_RFL8_NIR8_DUAL,
};

SensorInfo make_sensor_info(UDPProfileLidar profile, const std::string& fw) {
    auto info = *SensorInfo::from_default(LidarMode::_512x10);
    info.format.udp_profile_lidar = profile;
    info.image_rev = fw;
    return info;
}

bool frame_has_window(const SensorInfo& info) {
    for (const auto& ft : get_field_types(info))
        if (ft.name == ChanField::WINDOW) return true;
    return false;
}
}  // namespace

// fw_has_window_field duplicates the version rule of ouster-sdk
// get_field_types, make sure the two never drift apart.
TEST_F(PointCloudComposeTest, FwHasWindowFieldMatchesSdk) {
    const std::vector<std::string> fw_versions{
        "ousteros-image-prod-aries-v2.5.3+20230928",
        "ousteros-image-prod-aries-v3.1.0+20231215",
        "ousteros-image-prod-aries-v3.2.0+20250101",
        "ousteros-image-prod-aries-v3.2.1+20250301",
        "",  // unknown firmware version
    };
    for (auto profile : WINDOW_PROFILES) {
        for (const auto& fw : fw_versions) {
            auto info = make_sensor_info(profile, fw);
            EXPECT_EQ(PointCloudProcessorFactory::fw_has_window_field(info),
                      frame_has_window(info))
                << to_string(profile) << " fw=" << fw;
        }
    }
}

// FW 3.1 and older don't send WINDOW, the point cloud processor must not throw
// and should publish window as zero; FW 3.2+ should copy the WINDOW values.
TEST_F(PointCloudComposeTest, ScanToCloudHandlesFirmwareWithoutWindow) {
    const std::vector<std::pair<std::string, bool>> fw_versions{
        {"ousteros-image-prod-aries-v3.1.0+20231215", false},
        {"ousteros-image-prod-aries-v3.2.0+20250101", true},
    };
    const std::vector<std::string> point_types{"native", "original", "xyzir"};
    const uint8_t WINDOW_VALUE = 7;

    for (auto profile : WINDOW_PROFILES) {
        for (const auto& [fw, has_window] : fw_versions) {
            auto info = make_sensor_info(profile, fw);
            for (const auto& point_type : point_types) {
                SCOPED_TRACE(to_string(profile) + " fw=" + fw +
                             " point_type=" + point_type);
                PointCloudProcessor_OutputType output;
                auto processor =
                    PointCloudProcessorFactory::create_point_cloud_processor(
                        point_type, info, "os_lidar", false, true, true, 0,
                        1000000, 1, "", [&output](auto msgs) { output = msgs; });

                LidarFrame ls(std::make_shared<SensorInfo>(info));
                ASSERT_EQ(ls.has_field(ChanField::WINDOW), has_window);
                const auto samples = ls.w * ls.h;
                auto* range = ls.field<uint32_t>(ChanField::RANGE).data();
                std::fill(range, range + samples, 1000U);
                if (has_window) {
                    auto* window = ls.field<uint8_t>(ChanField::WINDOW).data();
                    std::fill(window, window + samples, WINDOW_VALUE);
                }

                ASSERT_NO_THROW(processor(ls, 0, rclcpp::Time()));
                ASSERT_EQ(output.size(),
                          static_cast<size_t>(info.num_returns()));

                if (point_type != "native") continue;
                // native clouds keep a stable layout regardless of firmware
                auto& msg = *output[0];
                sensor_msgs::PointCloud2ConstIterator<uint8_t> it(msg, "window");
                const uint8_t expected = has_window ? WINDOW_VALUE : 0;
                for (size_t i = 0; i < msg.width * msg.height; ++i, ++it)
                    ASSERT_EQ(*it, expected) << "i=" << i;
            }
        }
    }
}
