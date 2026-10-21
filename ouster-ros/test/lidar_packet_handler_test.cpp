#include <gtest/gtest.h>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <vector>

#include "../src/lidar_packet_handler.h"

using namespace ouster::sdk::core;
using namespace ouster_ros;
using namespace std::chrono_literals;

namespace {

constexpr uint32_t WIDTH = 512;
constexpr uint32_t HEIGHT = 16;
constexpr uint32_t COLS_PER_PACKET = 16;
constexpr uint32_t PACKETS_PER_FRAME = WIDTH / COLS_PER_PACKET;
constexpr uint32_t FPS = 10;
constexpr uint32_t INIT_ID = 7;
// one packet worth of columns, so that extrapolating a packet's reception time
// back to the first column of its scan gives exact values
constexpr uint64_t PACKET_PERIOD_NS =
    1'000'000'000ull * COLS_PER_PACKET / (WIDTH * FPS);

SensorInfo make_sensor_info() {
    SensorInfo info;
    info.format.pixels_per_column = HEIGHT;
    info.format.columns_per_packet = COLS_PER_PACKET;
    info.format.columns_per_frame = WIDTH;
    info.format.imu_measurements_per_packet = 8;
    info.format.imu_packets_per_frame = 1;
    info.format.pixel_shift_by_row = std::vector<int>(HEIGHT, 0);
    info.format.column_window = {0, static_cast<int>(WIDTH) - 1};
    info.format.udp_profile_lidar = UDPProfileLidar::RNG19_RFL8_SIG16_NIR16;
    // a profile whose IMU packets FrameBatcher would wait for
    info.format.udp_profile_imu = UDPProfileIMU::ACCEL32_GYRO32_NMEA;
    info.format.fps = FPS;
    info.config.lidar_mode = LidarMode(WIDTH, FPS);
    info.init_id = INIT_ID;
    return info;
}

// host timestamp of the first packet of a frame
uint64_t frame_host_ts(uint16_t frame_id) {
    return 1'000'000'000ull + frame_id * 100'000'000ull;
}

LidarPacket make_packet(const SensorInfo& info, uint16_t frame_id,
                        uint32_t packet_idx) {
    auto pf = std::make_shared<PacketFormat>(get_format(info));
    LidarPacket packet(pf);
    packet.buf.assign(pf->lidar_packet_size, 0);
    packet.host_timestamp =
        frame_host_ts(frame_id) + packet_idx * PACKET_PERIOD_NS;
    pf->set_frame_id(packet.buf.data(), frame_id);
    pf->set_init_id(packet.buf.data(), INIT_ID);
    for (uint32_t c = 0; c < COLS_PER_PACKET; ++c) {
        uint8_t* col = pf->nth_col(c, packet.buf.data());
        pf->set_col_measurement_id(col, packet_idx * COLS_PER_PACKET + c);
        pf->set_col_status(col, 0x01);
        pf->set_col_timestamp(col, packet.host_timestamp + c);
    }
    return packet;
}

// collects what the handler passes to its scan processors, which run on the
// handler's own processing thread
class ScanRecorder {
   public:
    LidarFrameProcessor processor() {
        return [this](const LidarFrame& ls, uint64_t, const rclcpp::Time& ts) {
            std::lock_guard<std::mutex> lock(mutex);
            frame_ids.push_back(ls.frame_id);
            stamps.push_back(ts.nanoseconds());
            cv.notify_all();
        };
    }

    bool wait_for_scans(size_t count) {
        std::unique_lock<std::mutex> lock(mutex);
        return cv.wait_for(lock, 2s, [&] { return frame_ids.size() >= count; });
    }

    std::mutex mutex;
    std::condition_variable cv;
    std::vector<int64_t> frame_ids;
    std::vector<int64_t> stamps;
};

}  // namespace

class LidarPacketHandlerTest : public ::testing::Test {
   protected:
    LidarPacketHandler::HandlerType make_handler(
        const std::string& timestamp_mode) {
        return LidarPacketHandler::create(info, {recorder.processor()},
                                          timestamp_mode, 0, 0.0f);
    }

    void feed_frame(const LidarPacketHandler::HandlerType& handler,
                    uint16_t frame_id, uint32_t first_packet = 0,
                    uint32_t end_packet = PACKETS_PER_FRAME) {
        for (uint32_t p = first_packet; p < end_packet; ++p)
            handler(make_packet(info, frame_id, p));
    }

    SensorInfo info = make_sensor_info();
    ScanRecorder recorder;
};

TEST_F(LidarPacketHandlerTest, ReleasesScanWithItsLastPacket) {
    auto handler = make_handler("TIME_FROM_INTERNAL_OSC");
    feed_frame(handler, 1);
    // no packet of the next scan is needed to release this one
    ASSERT_TRUE(recorder.wait_for_scans(1));
    EXPECT_EQ(recorder.frame_ids[0], 1);
}

TEST_F(LidarPacketHandlerTest, KeepsProducingScansAfterFrameIdGoesBack) {
    auto handler = make_handler("TIME_FROM_INTERNAL_OSC");
    // e.g. a virtual sensor that replays a recording in a loop
    for (uint16_t f : {100, 101, 102, 10, 11, 12}) feed_frame(handler, f);
    ASSERT_TRUE(recorder.wait_for_scans(6));
    EXPECT_EQ(recorder.frame_ids,
              (std::vector<int64_t>{100, 101, 102, 10, 11, 12}));
}

TEST_F(LidarPacketHandlerTest, RosTimeStampsScansWithTheirFirstPacket) {
    auto handler = make_handler("TIME_FROM_ROS_TIME");
    for (uint16_t f : {1, 2, 3}) {
        feed_frame(handler, f);
        ASSERT_TRUE(recorder.wait_for_scans(f));
    }
    EXPECT_EQ(recorder.stamps,
              (std::vector<int64_t>{static_cast<int64_t>(frame_host_ts(1)),
                                    static_cast<int64_t>(frame_host_ts(2)),
                                    static_cast<int64_t>(frame_host_ts(3))}));
}

TEST_F(LidarPacketHandlerTest, RosTimeStampsWhenTheLastPacketIsLost) {
    auto handler = make_handler("TIME_FROM_ROS_TIME");
    // without its last packet scan 1 is only released after a few packets of
    // scan 2 arrived, scan 2 must still be stamped from its first column
    feed_frame(handler, 1, 0, PACKETS_PER_FRAME - 1);
    feed_frame(handler, 2, 0, 8);
    ASSERT_TRUE(recorder.wait_for_scans(1));
    feed_frame(handler, 2, 8);
    ASSERT_TRUE(recorder.wait_for_scans(2));
    EXPECT_EQ(recorder.frame_ids, (std::vector<int64_t>{1, 2}));
    EXPECT_EQ(recorder.stamps,
              (std::vector<int64_t>{static_cast<int64_t>(frame_host_ts(1)),
                                    static_cast<int64_t>(frame_host_ts(2))}));
}
