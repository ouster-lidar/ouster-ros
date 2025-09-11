/**
 * @file test_sensor_diagnostics_tracker.cpp
 * @brief Test program to verify SensorDiagnosticsTracker functionality
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "ouster_ros/sensor_diagnostics_tracker.h"

namespace ouster_ros
{
namespace test
{

class SensorDiagnosticsTrackerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    clock_ = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    tracker_ = std::make_unique<ouster_ros::SensorDiagnosticsTracker>(
      "test_sensor", clock_, "test_hardware_id");
  }

  rclcpp::Clock::SharedPtr clock_;
  std::unique_ptr<ouster_ros::SensorDiagnosticsTracker> tracker_;

};

TEST_F(SensorDiagnosticsTrackerTest, InitialState)
{
  auto status = tracker_->get_current_status();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
  EXPECT_EQ(status.name, "test_sensor");
  EXPECT_EQ(status.hardware_id, "test_hardware_id");
  EXPECT_EQ(status.message, "Not initialized");
}

static std::string find_key_value(
  const std::vector<diagnostic_msgs::msg::KeyValue> & kvs, const std::string & key)
{
  for (const auto & kv : kvs) {
    if (kv.key == key) {
      return kv.value;
    }
  }
  return "Key not found: <" + key + ">";
}

TEST_F(SensorDiagnosticsTrackerTest, RecordLidarPacket)
{
  tracker_->record_lidar_packet();
  auto status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "Total LIDAR Packets"), std::to_string(1));

  for (size_t i = 0; i < 99; ++i) {
    tracker_->record_lidar_packet();
  }
  status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "Total LIDAR Packets"), std::to_string(100));
}

TEST_F(SensorDiagnosticsTrackerTest, RecordImuPacket)
{
  tracker_->record_imu_packet();
  auto status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "Total IMU Packets"), std::to_string(1));

  tracker_->record_imu_packet();
  tracker_->record_imu_packet();
  status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "Total IMU Packets"), std::to_string(3));
}

TEST_F(SensorDiagnosticsTrackerTest, IncrementPollClientErrors)
{
  tracker_->increment_poll_client_errors();
  auto status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "Poll Client Errors"), std::to_string(1));
}

TEST_F(SensorDiagnosticsTrackerTest, IncrementLidarPacketErrors)
{
  tracker_->increment_lidar_packet_errors();
  auto status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "LIDAR Packet Errors"), std::to_string(1));
}

TEST_F(SensorDiagnosticsTrackerTest, IncrementImuPacketErrors)
{
  tracker_->increment_imu_packet_errors();
  auto status = tracker_->get_current_status();

  EXPECT_EQ(find_key_value(status.values, "IMU Packet Errors"), std::to_string(1));
}

TEST_F(SensorDiagnosticsTrackerTest, NotifyResetSensor)
{
  tracker_->notify_reset_sensor();
  auto debug_context = tracker_->get_debug_context("test_host", true);

  EXPECT_EQ(debug_context["Sensor Hostname"], "test_host");
  EXPECT_EQ(debug_context["Sensor Connection Active"], "true");
}

TEST_F(SensorDiagnosticsTrackerTest, UpdateStatus)
{
  tracker_->update_status("OK", diagnostic_msgs::msg::DiagnosticStatus::OK);
  auto status = tracker_->get_current_status();
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "OK");
}

TEST_F(SensorDiagnosticsTrackerTest, CreateDiagnosticStatus)
{
  std::map<std::string, std::string> debug_context = {{"key1", "value1"}, {"key2", "value2"}};
  auto status = tracker_->create_diagnostic_status(
    "Test message", diagnostic_msgs::msg::DiagnosticStatus::WARN, debug_context);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(status.name, "test_sensor");
  EXPECT_EQ(status.hardware_id, "test_hardware_id");
  EXPECT_EQ(status.message, "Test message");

  // The implementation includes all sensor metrics plus debug context
  // Expected: 14 sensor metrics + 2 debug context = 16 total values
  ASSERT_GE(status.values.size(), 14);

  bool found_key1 = false, found_key2 = false;
  for (const auto & kv : status.values) {
    if (kv.key == "key1" && kv.value == "value1") {
      found_key1 = true;
    }
    if (kv.key == "key2" && kv.value == "value2") {
      found_key2 = true;
    }
  }
  EXPECT_TRUE(found_key1);
  EXPECT_TRUE(found_key2);
}

TEST_F(SensorDiagnosticsTrackerTest, UpdateMetadata)
{
  ouster::sensor::sensor_info info;
  info.sn = "test_serial";
  info.fw_rev = "test_firmware";

  info.beam_azimuth_angles.clear();
  info.beam_altitude_angles.clear();

  info.format.columns_per_frame = 1024;
  info.format.pixels_per_column = 64;

  tracker_->update_metadata(info);

  auto status =
    tracker_->create_diagnostic_status("test", diagnostic_msgs::msg::DiagnosticStatus::OK);

  bool found_serial = false, found_firmware = false;
  for (const auto & kv : status.values) {
    if (kv.key == "Sensor Serial Number" && kv.value == "test_serial") {
      found_serial = true;
    }
    if (kv.key == "Sensor Firmware Version" && kv.value == "test_firmware") {
      found_firmware = true;
    }
  }
  EXPECT_TRUE(found_serial);
  EXPECT_TRUE(found_firmware);
}

TEST_F(SensorDiagnosticsTrackerTest, TestDiagnosticStatusFormat)
{
  std::map<std::string, std::string> debug_context = {{"Test Key", "Test Value"}};

  auto status = tracker_->create_diagnostic_status(
    "Test Message", diagnostic_msgs::msg::DiagnosticStatus::ERROR, debug_context);

  EXPECT_EQ(status.name, "test_sensor");
  EXPECT_EQ(status.hardware_id, "test_hardware_id");
  EXPECT_EQ(status.message, "Test Message");
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);

  std::vector<std::string> required_fields = {
    "Last Update",
    "Last Sensor Reset",
    "Total Sensor Resets",
    "Sensor Uptime (s)",
    "Last successful LiDAR frame",
    "Last unsuccessful LiDAR frame",
    "Total LIDAR Packets",
    "LIDAR Packet Errors",
    "Last successful IMU frame",
    "Last unsuccessful IMU frame",
    "Total IMU Packets",
    "IMU Packet Errors",
    "Poll Client Errors",
    "Last Client Error",
    "Test Key"  // From debug context
  };

  for (const auto & field : required_fields) {
    bool found = false;
    for (const auto & kv : status.values) {
      if (kv.key == field) {
        found = true;
        break;
      }
    }
    EXPECT_TRUE(found) << "Missing field: " << field;
  }
}

}  // namespace test
}  // namespace ouster_ros