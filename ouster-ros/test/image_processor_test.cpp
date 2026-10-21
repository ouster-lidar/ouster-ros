#include <gtest/gtest.h>
#include <ouster/core/lidar_frame.h>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "../src/image_processor.h"

class ImageProcessorTest : public ::testing::Test {};

using namespace ouster_ros;
using namespace ouster::sdk::core;

namespace {
std::shared_ptr<SensorInfo> make_sensor_info(UDPProfileLidar profile,
                                             const std::string& fw) {
    auto info = SensorInfo::from_default(LidarMode::_512x10);
    info->format.udp_profile_lidar = profile;
    info->image_rev = fw;
    return info;
}

const std::string FW_3_1 = "ousteros-image-prod-aries-v3.1.0+20231215";
const std::string FW_3_2 = "ousteros-image-prod-aries-v3.2.0+20250101";
}  // namespace

TEST_F(ImageProcessorTest, HasWindowFieldDependsOnProfileAndFirmware) {
    EXPECT_TRUE(ImageProcessor::has_window_field(*make_sensor_info(
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, FW_3_2)));
    EXPECT_TRUE(ImageProcessor::has_window_field(*make_sensor_info(
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL, FW_3_2)));
    EXPECT_FALSE(ImageProcessor::has_window_field(*make_sensor_info(
        UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, FW_3_1)));
    EXPECT_FALSE(ImageProcessor::has_window_field(*make_sensor_info(
        UDPProfileLidar::LEGACY, FW_3_2)));
}

TEST_F(ImageProcessorTest, WindowImageOnlyProducedWhenFieldPresent) {
    for (const auto& fw : {FW_3_1, FW_3_2}) {
        auto info = make_sensor_info(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16, fw);
        ImageProcessor::OutputType output;
        auto processor = ImageProcessor::create(
            *info, "os_lidar", "", [&output](auto msgs) { output = msgs; });
        LidarFrame ls(info);
        processor(ls, 0, rclcpp::Time());
        EXPECT_EQ(output.count(ChanField::WINDOW), fw == FW_3_2 ? 1U : 0U)
            << fw;
        EXPECT_EQ(output.count(ChanField::RANGE), 1U) << fw;
    }
}

TEST_F(ImageProcessorTest, WindowImageIsDestaggeredAndScaled) {
    auto info =
        make_sensor_info(UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL, FW_3_2);
    const size_t H = info->format.pixels_per_column;
    const size_t W = info->format.columns_per_frame;
    const auto& px_offset = info->format.pixel_shift_by_row;

    ImageProcessor::OutputType output;
    auto processor = ImageProcessor::create(
        *info, "os_lidar", "", [&output](auto msgs) { output = msgs; });

    LidarFrame ls(info);
    auto* wd = ls.field<uint8_t>(ChanField::WINDOW).data();
    for (size_t u = 0; u < H; ++u)
        for (size_t v = 0; v < W; ++v) wd[u * W + v] = (u + v) % 256;

    processor(ls, 0, rclcpp::Time());
    ASSERT_EQ(output.count(ChanField::WINDOW), 1U);
    const auto& msg = *output[ChanField::WINDOW];
    EXPECT_EQ(msg.encoding, sensor_msgs::image_encodings::MONO16);
    ASSERT_EQ(msg.width, W);
    ASSERT_EQ(msg.height, H);

    const auto* px = reinterpret_cast<const uint16_t*>(msg.data.data());
    for (size_t u = 0; u < H; ++u) {
        for (size_t v = 0; v < W; ++v) {
            const size_t vv = (v + W - px_offset[u]) % W;
            ASSERT_EQ(px[u * W + v], wd[u * W + vv] * 257)
                << "u=" << u << " v=" << v;
        }
    }
}
