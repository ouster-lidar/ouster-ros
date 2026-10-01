#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "../src/zone_packet_handler.h"

using namespace ouster_ros;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::ZonePacket;
using ouster_sensor_msgs::msg::ZoneState;

namespace {

std::string read_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    EXPECT_TRUE(file.is_open()) << "failed to open " << path;
    return std::string(std::istreambuf_iterator<char>(file),
                       std::istreambuf_iterator<char>());
}

}  // namespace

// The fixtures below were captured from zm9.pcap/zm9_0.json, a recording of an
// OS sensor running the RNG19_RFL8_SIG16_NIR16_ZONE16 profile with four live
// occupancy zones, one of which (zone 1) stays triggered for the whole
// recording while the other three stay clear.
class ZonePacketHandlerTest : public ::testing::Test {
   protected:
    void SetUp() override {
        const std::string data_dir = OUSTER_ROS_TEST_DATA_DIR;
        info = SensorInfo(read_file(data_dir + "/zm9_0.json"));
        packet_format =
            std::make_shared<PacketFormat>(ouster::sdk::core::get_format(info));

        auto blob = read_file(data_dir + "/zm9_zone_packets.bin");
        const auto packet_size = packet_format->zone_packet_size;
        ASSERT_GT(packet_size, 0U);
        ASSERT_EQ(blob.size() % packet_size, 0U);
        for (size_t offset = 0; offset < blob.size(); offset += packet_size) {
            ZonePacket packet(static_cast<int>(packet_size));
            packet.format = packet_format;
            std::memcpy(packet.buf.data(), blob.data() + offset, packet_size);
            packets.push_back(std::move(packet));
        }
    }

    SensorInfo info;
    std::shared_ptr<PacketFormat> packet_format;
    std::vector<ZonePacket> packets;
};

TEST_F(ZonePacketHandlerTest, MetadataAdvertisesZoneMonitoring) {
    EXPECT_TRUE(info.format.zone_monitoring_enabled);
    EXPECT_EQ(info.config.udp_port_zm.value_or(0), 7504);
    EXPECT_EQ(packet_format->zone_packet_size, 680U);
    EXPECT_EQ(packets.size(), 46U);
}

TEST_F(ZonePacketHandlerTest, ZoneLabelsAreCollectedFromTheZoneSet) {
    auto zone_labels = get_zone_labels(info);
    // zm9_0.json defines four zones, all of them with an empty label
    ASSERT_EQ(zone_labels.size(), 4U);
    for (uint8_t id = 0; id < 4; ++id) {
        ASSERT_EQ(zone_labels.count(id), 1U);
        EXPECT_EQ(zone_labels[id], "");
    }
}

TEST_F(ZonePacketHandlerTest, EveryPacketYieldsAllSixteenZoneSlots) {
    auto handler = ZonePacketHandler::create(info, "os_sensor", "", 0);
    ASSERT_TRUE(handler != nullptr);

    for (const auto& packet : packets) {
        auto msg = handler(packet);
        EXPECT_EQ(msg.header.frame_id, "os_sensor");
        ASSERT_EQ(msg.zones.size(), 16U);
        // the four configured zones occupy the first four slots, the rest are
        // empty slots
        for (size_t i = 0; i < 4; ++i) {
            EXPECT_TRUE(msg.zones[i].live) << "slot " << i;
            EXPECT_EQ(msg.zones[i].id, i);
            EXPECT_EQ(msg.zones[i].trigger_type,
                      ZoneState::TRIGGER_TYPE_OCCUPANCY);
            EXPECT_EQ(msg.zones[i].error_flags, 0);
            EXPECT_GT(msg.zones[i].max_count, 0U);
        }
        for (size_t i = 4; i < msg.zones.size(); ++i) {
            EXPECT_FALSE(msg.zones[i].live) << "slot " << i;
            EXPECT_EQ(msg.zones[i].count, 0U);
            EXPECT_EQ(msg.zones[i].trigger_status,
                      ZoneState::TRIGGER_STATUS_DEASSERTED);
        }
    }
}

TEST_F(ZonePacketHandlerTest, TriggerStateMatchesTheRecording) {
    auto handler = ZonePacketHandler::create(info, "os_sensor", "", 0);

    uint32_t previous_triggered_frames = 0;
    for (size_t p = 0; p < packets.size(); ++p) {
        auto msg = handler(packets[p]);

        // zone 1 is asserted throughout the recording and its consecutive
        // trigger count keeps growing
        const auto& triggered = msg.zones[1];
        EXPECT_EQ(triggered.trigger_status, ZoneState::TRIGGER_STATUS_ASSERTED)
            << "packet " << p;
        EXPECT_GT(triggered.triggered_frames, previous_triggered_frames)
            << "packet " << p;
        EXPECT_GT(triggered.count, 0U) << "packet " << p;
        EXPECT_LE(triggered.min_range, triggered.mean_range) << "packet " << p;
        EXPECT_LE(triggered.mean_range, triggered.max_range) << "packet " << p;
        previous_triggered_frames = triggered.triggered_frames;

        // the remaining zones never assert; zone 2 does briefly accumulate a
        // single triggered frame, which stays below the 2 frame threshold the
        // zone set configures
        for (size_t i : {size_t{0}, size_t{2}, size_t{3}}) {
            EXPECT_EQ(msg.zones[i].trigger_status,
                      ZoneState::TRIGGER_STATUS_DEASSERTED)
                << "packet " << p << " zone " << i;
            EXPECT_LT(msg.zones[i].triggered_frames, 2U)
                << "packet " << p << " zone " << i;
        }
    }
    // 46 frames at 10 Hz worth of consecutive triggering
    EXPECT_EQ(previous_triggered_frames, 3083U);
}

TEST_F(ZonePacketHandlerTest, ZoneSetHashIsStableAcrossTheRecording) {
    auto handler = ZonePacketHandler::create(info, "os_sensor", "", 0);

    auto expected = handler(packets.front()).zoneset_hash;
    bool all_zeros = std::all_of(expected.begin(), expected.end(),
                                 [](uint8_t b) { return b == 0; });
    EXPECT_FALSE(all_zeros);
    for (const auto& packet : packets)
        EXPECT_EQ(handler(packet).zoneset_hash, expected);
}

TEST_F(ZonePacketHandlerTest, SensorTimestampsAreUsedByDefault) {
    auto handler = ZonePacketHandler::create(info, "os_sensor", "", 0);

    uint64_t previous_ts = 0;
    for (const auto& packet : packets) {
        auto msg = handler(packet);
        EXPECT_EQ(msg.timestamp, packet.timestamp());
        EXPECT_EQ(rclcpp::Time(msg.header.stamp).nanoseconds(),
                  static_cast<int64_t>(packet.timestamp()));
        EXPECT_GT(msg.timestamp, previous_ts);
        previous_ts = msg.timestamp;
    }
}

TEST_F(ZonePacketHandlerTest, PtpTimestampModeAppliesTheUtcTaiOffset) {
    const int64_t offset_ns = -37000000000;
    auto handler = ZonePacketHandler::create(info, "os_sensor",
                                             "TIME_FROM_PTP_1588", offset_ns);
    for (const auto& packet : packets) {
        auto msg = handler(packet);
        // the reported measurement timestamp stays untouched, only the header
        // stamp is shifted
        EXPECT_EQ(msg.timestamp, packet.timestamp());
        EXPECT_EQ(rclcpp::Time(msg.header.stamp).nanoseconds(),
                  static_cast<int64_t>(packet.timestamp()) + offset_ns);
    }
}

TEST_F(ZonePacketHandlerTest, RosTimestampModeUsesTheHostReceiveTime) {
    auto handler =
        ZonePacketHandler::create(info, "os_sensor", "TIME_FROM_ROS_TIME", 0);

    auto packet = packets.front();
    packet.host_timestamp = 1234567890123456789ULL;
    auto msg = handler(packet);
    EXPECT_EQ(rclcpp::Time(msg.header.stamp).nanoseconds(),
              static_cast<int64_t>(packet.host_timestamp));
    EXPECT_EQ(msg.timestamp, packet.timestamp());
}
