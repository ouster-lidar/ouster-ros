#include <gtest/gtest.h>

#include <fstream>
#include <string>
#include <vector>

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on
#include "../src/zone_marker_handler.h"
#include "../src/zone_packet_handler.h"

using namespace ouster_ros;
using ouster::sdk::core::Coord;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::Stl;
using ouster::sdk::core::Triangle;
using ouster::sdk::core::Zone;
using ouster::sdk::core::ZonePacket;
using ouster::sdk::core::ZoneSet;
using ouster_sensor_msgs::msg::ZoneState;
using visualization_msgs::msg::Marker;

namespace {

std::string read_file(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    EXPECT_TRUE(file.is_open()) << "failed to open " << path;
    return std::string(std::istreambuf_iterator<char>(file),
                       std::istreambuf_iterator<char>());
}

// a single right triangle with legs of length 2, in an otherwise-arbitrary
// winding order, expressed as an ASCII STL blob
std::vector<uint8_t> single_triangle_stl(const std::string& name) {
    std::string text =
        "solid " + name +
        "\n"
        "facet normal 0 0 1\n"
        "  outer loop\n"
        "    vertex 0 0 0\n"
        "    vertex 2 0 0\n"
        "    vertex 0 2 0\n"
        "  endloop\n"
        "endfacet\n"
        "endsolid " +
        name + "\n";
    return std::vector<uint8_t>(text.begin(), text.end());
}

const Marker* find_marker(const visualization_msgs::msg::MarkerArray& markers,
                          const std::string& ns, int id) {
    for (const auto& m : markers.markers) {
        if (m.ns == ns && m.id == id) return &m;
    }
    return nullptr;
}

ZoneState make_zone_state(uint8_t id, bool live, uint8_t trigger_status,
                          uint8_t error_flags = 0, uint32_t count = 0) {
    ZoneState z;
    z.live = live;
    z.id = id;
    z.trigger_status = trigger_status;
    z.error_flags = error_flags;
    z.count = count;
    return z;
}

}  // namespace

// Synthetic-geometry tests: exercise the BODY -> sensor frame coordinate
// transform directly with a hand-built, non-identity sensor_to_body_transform
// so the math is validated independently of the (identity-transform) pcap
// fixture used by the end-to-end tests below.
class ZoneMarkerHandlerSyntheticTest : public ::testing::Test {
   protected:
    void SetUp() override {
        info.zone_set = ZoneSet{};

        // zone 0: BODY-frame STL, translated 1m/2m/3m from the sensor origin
        Zone body_zone;
        body_zone.stl = Stl(single_triangle_stl("body"));
        body_zone.stl->coordinate_frame = Stl::CoordinateFrame::BODY;
        body_zone.label = "body_zone";
        info.zone_set->zones.emplace(0, body_zone);

        // zone 1: SENSOR-frame STL, should pass through untransformed
        Zone sensor_zone;
        sensor_zone.stl = Stl(single_triangle_stl("sensor"));
        sensor_zone.stl->coordinate_frame = Stl::CoordinateFrame::SENSOR;
        sensor_zone.label = "sensor_zone";
        info.zone_set->zones.emplace(1, sensor_zone);

        // sensor_to_body_transform: identity rotation, translated by
        // (1, 2, 3) meters, i.e. p_body = p_sensor + (1, 2, 3)
        info.zone_set->sensor_to_body_transform =
            ouster::sdk::core::mat4d::Identity();
        info.zone_set->sensor_to_body_transform(0, 3) = 1.0;
        info.zone_set->sensor_to_body_transform(1, 3) = 2.0;
        info.zone_set->sensor_to_body_transform(2, 3) = 3.0;
    }

    SensorInfo info;
};

TEST_F(ZoneMarkerHandlerSyntheticTest, BodyFrameGeometryIsTransformedToSensorFrame) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));
    status.zones.push_back(make_zone_state(1, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);

    auto* body_mesh = find_marker(markers, "ouster_zones", 0);
    ASSERT_NE(body_mesh, nullptr);
    ASSERT_EQ(body_mesh->points.size(), 3U);
    // p_sensor = p_body - (1, 2, 3), since sensor_to_body_transform is a pure
    // translation of (1, 2, 3) from sensor to body frame
    EXPECT_NEAR(body_mesh->points[0].x, 0.0 - 1.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[0].y, 0.0 - 2.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[0].z, 0.0 - 3.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[1].x, 2.0 - 1.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[1].y, 0.0 - 2.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[2].x, 0.0 - 1.0, 1e-4);
    EXPECT_NEAR(body_mesh->points[2].y, 2.0 - 2.0, 1e-4);
}

TEST_F(ZoneMarkerHandlerSyntheticTest, SensorFrameGeometryPassesThroughUnchanged) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));
    status.zones.push_back(make_zone_state(1, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);

    auto* sensor_mesh = find_marker(markers, "ouster_zones", 1);
    ASSERT_NE(sensor_mesh, nullptr);
    ASSERT_EQ(sensor_mesh->points.size(), 3U);
    EXPECT_NEAR(sensor_mesh->points[0].x, 0.0, 1e-4);
    EXPECT_NEAR(sensor_mesh->points[0].y, 0.0, 1e-4);
    EXPECT_NEAR(sensor_mesh->points[1].x, 2.0, 1e-4);
    EXPECT_NEAR(sensor_mesh->points[2].y, 2.0, 1e-4);
}

TEST_F(ZoneMarkerHandlerSyntheticTest, MarkersUseTheGivenFrameAndStamp) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.header.stamp = rclcpp::Time(123, 456);
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);
    ASSERT_FALSE(markers.markers.empty());
    for (const auto& m : markers.markers) {
        EXPECT_EQ(m.header.frame_id, "os_sensor");
        EXPECT_EQ(m.header.stamp, status.header.stamp);
    }
}

TEST_F(ZoneMarkerHandlerSyntheticTest, TriggeredZoneIsRedAndClearZoneIsGreen) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_ASSERTED));
    status.zones.push_back(make_zone_state(1, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);

    auto* triggered = find_marker(markers, "ouster_zones", 0);
    auto* clear = find_marker(markers, "ouster_zones", 1);
    ASSERT_NE(triggered, nullptr);
    ASSERT_NE(clear, nullptr);

    EXPECT_GT(triggered->color.r, 0.5f);
    EXPECT_EQ(triggered->color.g, 0.0f);
    EXPECT_EQ(triggered->action, Marker::ADD);

    EXPECT_GT(clear->color.g, 0.5f);
    EXPECT_EQ(clear->color.r, 0.0f);
    EXPECT_EQ(clear->action, Marker::ADD);

    auto* triggered_label = find_marker(markers, "ouster_zone_labels", 0);
    ASSERT_NE(triggered_label, nullptr);
    EXPECT_NE(triggered_label->text.find("TRIGGERED"), std::string::npos);

    auto* clear_label = find_marker(markers, "ouster_zone_labels", 1);
    ASSERT_NE(clear_label, nullptr);
    EXPECT_NE(clear_label->text.find("clear"), std::string::npos);
}

TEST_F(ZoneMarkerHandlerSyntheticTest, ErrorFlagsOverrideTriggerColorToOrange) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_ASSERTED,
                                           /*error_flags=*/1));

    auto markers = handler(status);
    auto* degraded = find_marker(markers, "ouster_zones", 0);
    ASSERT_NE(degraded, nullptr);
    EXPECT_GT(degraded->color.r, 0.5f);
    EXPECT_GT(degraded->color.g, 0.0f);  // orange, not pure red
}

TEST_F(ZoneMarkerHandlerSyntheticTest, NonLiveZoneIsDeleted) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    // zone 1 is absent from the status entirely, e.g. after a metadata
    // update that reduced the number of live zones
    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);

    auto* absent_mesh = find_marker(markers, "ouster_zones", 1);
    auto* absent_label = find_marker(markers, "ouster_zone_labels", 1);
    ASSERT_NE(absent_mesh, nullptr);
    ASSERT_NE(absent_label, nullptr);
    EXPECT_EQ(absent_mesh->action, Marker::DELETE);
    EXPECT_EQ(absent_label->action, Marker::DELETE);
}

TEST_F(ZoneMarkerHandlerSyntheticTest, MeshMarkersAreWellFormedTriangleLists) {
    auto handler = ZoneMarkerHandler::create(info, "os_sensor");

    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));
    status.zones.push_back(make_zone_state(1, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);
    for (const auto& m : markers.markers) {
        if (m.ns != "ouster_zones") continue;
        EXPECT_EQ(m.type, Marker::TRIANGLE_LIST);
        EXPECT_EQ(m.points.size() % 3, 0U);
        EXPECT_FALSE(m.points.empty());
    }
}

TEST_F(ZoneMarkerHandlerSyntheticTest, ZoneWithoutStlIsSkipped) {
    // a zone without any geometry (e.g. ZRB-only) shouldn't produce a marker
    info.zone_set->zones.erase(1);
    Zone geometry_less_zone;  // no stl, no zrb
    info.zone_set->zones.emplace(1, geometry_less_zone);

    auto handler = ZoneMarkerHandler::create(info, "os_sensor");
    ouster_sensor_msgs::msg::ZoneStatus status;
    status.zones.push_back(make_zone_state(0, true, ZoneState::TRIGGER_STATUS_DEASSERTED));
    status.zones.push_back(make_zone_state(1, true, ZoneState::TRIGGER_STATUS_DEASSERTED));

    auto markers = handler(status);
    EXPECT_EQ(find_marker(markers, "ouster_zones", 1), nullptr);
    EXPECT_NE(find_marker(markers, "ouster_zones", 0), nullptr);
}

// End-to-end tests reusing the zm9.pcap/zm9_0.json fixtures also used by
// zone_packet_handler_test.cpp, running actual recorded zone packets through
// the same ZonePacketHandler -> ZoneMarkerHandler pipeline the driver uses.
class ZoneMarkerHandlerRecordingTest : public ::testing::Test {
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
        ASSERT_FALSE(packets.empty());
    }

    SensorInfo info;
    std::shared_ptr<PacketFormat> packet_format;
    std::vector<ZonePacket> packets;
};

TEST_F(ZoneMarkerHandlerRecordingTest, OneMeshAndLabelMarkerPerConfiguredZone) {
    auto status_handler = ZonePacketHandler::create(info, "os_sensor", "", 0);
    auto marker_handler = ZoneMarkerHandler::create(info, "os_sensor");

    auto status = status_handler(packets.front());
    auto markers = marker_handler(status);

    // zm9_0.json defines 4 zones, each with STL geometry
    for (uint8_t id = 0; id < 4; ++id) {
        auto* mesh = find_marker(markers, "ouster_zones", id);
        auto* label = find_marker(markers, "ouster_zone_labels", id);
        ASSERT_NE(mesh, nullptr) << "zone " << static_cast<int>(id);
        ASSERT_NE(label, nullptr) << "zone " << static_cast<int>(id);
        EXPECT_EQ(mesh->action, Marker::ADD);
        EXPECT_FALSE(mesh->points.empty());
    }
}

TEST_F(ZoneMarkerHandlerRecordingTest, MarkerColorsTrackTriggerStateAcrossTheRecording) {
    auto status_handler = ZonePacketHandler::create(info, "os_sensor", "", 0);
    auto marker_handler = ZoneMarkerHandler::create(info, "os_sensor");

    for (const auto& packet : packets) {
        auto status = status_handler(packet);
        auto markers = marker_handler(status);

        // zone 1 is asserted throughout the recording -> red mesh marker
        auto* triggered = find_marker(markers, "ouster_zones", 1);
        ASSERT_NE(triggered, nullptr);
        EXPECT_GT(triggered->color.r, 0.5f);
        EXPECT_EQ(triggered->color.g, 0.0f);

        // the rest stay clear -> green mesh markers
        for (uint8_t id : {uint8_t{0}, uint8_t{2}, uint8_t{3}}) {
            auto* clear = find_marker(markers, "ouster_zones", id);
            ASSERT_NE(clear, nullptr) << "zone " << static_cast<int>(id);
            EXPECT_GT(clear->color.g, 0.5f);
            EXPECT_EQ(clear->color.r, 0.0f);
        }
    }
}
