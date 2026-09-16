// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <cstdlib>
#include <stdexcept>

#include "../src/lidar_packet_handler.h"

TEST(LidarPacketHandlerMetadataTest, AcceptsDataFormatWithoutLidarMode) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    // Isolate construction so a prematurely started std::thread cannot abort
    // the entire test process if a later initialization step throws.
    EXPECT_EXIT(
        {
            auto info = ouster::sdk::core::default_sensor_info(
                ouster::sdk::core::LidarMode::_512x10);
            info.config.lidar_mode.reset();
            try {
                ouster_ros::LidarPacketHandler handler(info, {}, "", 0, 0.0f);
            } catch (...) {
                std::_Exit(1);
            }
            std::_Exit(0);
        },
        ::testing::ExitedWithCode(0), "");
}

TEST(LidarPacketHandlerMetadataTest, RejectsInvalidTimingWithoutTerminating) {
    ::testing::FLAGS_gtest_death_test_style = "threadsafe";
    EXPECT_EXIT(
        {
            auto info = ouster::sdk::core::default_sensor_info(
                ouster::sdk::core::LidarMode::_512x10);
            info.config.lidar_mode.reset();
            info.format.fps = 0;
            try {
                ouster_ros::LidarPacketHandler handler(info, {}, "", 0, 0.0f);
            } catch (const std::invalid_argument&) {
                std::_Exit(0);
            } catch (...) {
                std::_Exit(2);
            }
            std::_Exit(1);
        },
        ::testing::ExitedWithCode(0), "");
}
