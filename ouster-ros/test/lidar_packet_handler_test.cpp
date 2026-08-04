/**
 * Copyright (c) 2018-2026, Ouster, Inc.
 * All rights reserved.
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "../src/lidar_packet_handler.h"

TEST(LidarPacketHandlerTimestampTest, InterpolatesEpochNanosecondsExactly) {
    constexpr uint64_t base = 1700000000000000123ULL;

    EXPECT_EQ(base + 1021000ULL,
              linear_interpolate(3, base, 1027, base + 1024000ULL, 1024));
}

TEST(LidarPacketHandlerTimestampTest, InterpolatesDecreasingValuesExactly) {
    constexpr uint64_t base = 1700000000001024123ULL;

    EXPECT_EQ(base - 1021000ULL,
              linear_interpolate(3, base, 1027, base - 1024000ULL, 1024));
}

TEST(OusterRosTimestampTest, AppliesSignedOffsetsWithoutOverflow) {
    EXPECT_EQ(0U,
              ouster_ros::impl::ts_safe_offset_add(
                  1U, std::numeric_limits<int64_t>::min()));
    EXPECT_EQ(std::numeric_limits<uint64_t>::max(),
              ouster_ros::impl::ts_safe_offset_add(
                  std::numeric_limits<uint64_t>::max(), 1));
    EXPECT_EQ(42U, ouster_ros::impl::ts_safe_offset_add(79U, -37));
}
