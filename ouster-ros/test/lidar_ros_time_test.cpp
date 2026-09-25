// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>

#include "../src/lidar_packet_handler.h"
#include <ouster/impl/packet_writer.h>

namespace {

using ouster::sdk::core::UDPProfileLidar;

class LidarRosTimeTest : public ::testing::TestWithParam<UDPProfileLidar> {
   protected:
    void SetUp() override {
        info = ouster::sdk::core::default_sensor_info(
            ouster::sdk::core::LidarMode::_512x10);
        info.format.udp_profile_lidar = GetParam();
        handler = ouster_ros::LidarPacketHandler::create(
            info,
            {[this](const ouster::sdk::core::LidarScan& scan, uint64_t,
                    const rclcpp::Time& stamp) {
                std::lock_guard<std::mutex> lock(mutex);
                stamps[scan.frame_id] = stamp.nanoseconds();
                ready.notify_all();
            }},
            "TIME_FROM_ROS_TIME", 0, 0.0f);
    }

    void send_packet(uint32_t frame, int packet_index, uint64_t receive_ns) {
        const ouster::sdk::core::impl::PacketWriter writer{
            ouster::sdk::core::get_format(info)};
        ouster::sdk::core::LidarPacket packet(writer.lidar_packet_size);
        packet.host_timestamp = receive_ns;
        writer.set_frame_id(packet.buf.data(), frame);
        for (int i = 0; i < writer.columns_per_packet; ++i) {
            const int column_index = packet_index * writer.columns_per_packet + i;
            auto* column = writer.nth_col(i, packet.buf.data());
            writer.set_col_measurement_id(column, column_index);
            writer.set_col_status(column, 1);
            writer.set_col_timestamp(
                column, 1'000'000'000ULL + frame * 100'000'000ULL +
                            column_index * 100'000'000ULL / 512);
        }
        handler(packet);
    }

    void send_packets(uint32_t frame, uint64_t start_ns, int begin = 0,
                      int end = 32) {
        for (int i = begin; i < end; ++i) {
            send_packet(frame, i, start_ns + i * 3'125'000ULL);
        }
    }

    void expect_stamp(uint32_t frame, int64_t expected_ns) {
        std::unique_lock<std::mutex> lock(mutex);
        ASSERT_TRUE(ready.wait_for(lock, std::chrono::seconds(5), [&] {
            return stamps.count(frame) != 0;
        })) << "No completed scan for frame " << frame;
        EXPECT_EQ(stamps.at(frame), expected_ns);
    }

    ouster::sdk::core::SensorInfo info;
    std::mutex mutex;
    std::condition_variable ready;
    std::map<uint32_t, int64_t> stamps;
    // Destroy the handler (joining its worker) before the callback state.
    ouster_ros::LidarPacketHandler::HandlerType handler;
};

TEST_P(LidarRosTimeTest, ConsecutiveCompleteFramesUseTheirOwnArrivalTimes) {
    send_packets(41, 2'000'000'000);
    expect_stamp(41, 2'000'000'000);
    send_packets(42, 2'100'000'000);
    expect_stamp(42, 2'100'000'000);
    send_packets(43, 2'200'000'000);
    expect_stamp(43, 2'200'000'000);
}

TEST_P(LidarRosTimeTest, MissingLeadingPacketsExtrapolateToColumnZero) {
    send_packets(41, 2'000'000'000, 2);
    send_packet(42, 0, 2'100'000'000);
    expect_stamp(41, 2'000'000'000);
    send_packets(42, 2'100'000'000, 1);
    expect_stamp(42, 2'100'000'000);
}

TEST_P(LidarRosTimeTest, RolloverUsesTheCompletedFramesOwnArrivalTimes) {
    send_packets(41, 2'000'000'000, 0, 31);
    send_packet(42, 0, 2'100'000'000);
    expect_stamp(41, 2'000'000'000);
    send_packets(42, 2'100'000'000, 1);
    expect_stamp(42, 2'100'000'000);
}

TEST_P(LidarRosTimeTest, MissingArrivalTimesUseCompletionFallback) {
    for (int i = 0; i < 32; ++i) send_packet(41, i, 0);
    send_packet(42, 0, 2'100'000'000);
    expect_stamp(41, 2'000'000'000);
}

TEST_P(LidarRosTimeTest, LeadingPacketExtrapolationClampsAtClockStart) {
    send_packet(41, 1, 1);
    send_packet(42, 0, 200'000'000);
    expect_stamp(41, 0);
}

TEST_P(LidarRosTimeTest, MissingArrivalFallbackClampsAtClockStart) {
    for (int i = 0; i < 32; ++i) send_packet(41, i, 0);
    send_packet(42, 0, 1);
    expect_stamp(41, 0);
}

INSTANTIATE_TEST_SUITE_P(
    LegacyAndDualReturn, LidarRosTimeTest,
    ::testing::Values(UDPProfileLidar::LEGACY,
                      UDPProfileLidar::RNG19_RFL8_SIG16_NIR16_DUAL));

}  // namespace
