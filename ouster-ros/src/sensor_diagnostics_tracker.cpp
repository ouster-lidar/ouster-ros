/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_diagnostics_tracker.cpp
 * @brief Implementation of diagnostic tracking functionality for Ouster sensors with message analysis
 */

#include "ouster_ros/sensor_diagnostics_tracker.h"
#include <algorithm>

namespace ouster_ros
{

void SensorDiagnosticsState::record_lidar_packet()
{
  last_successful_lidar_frame_ = clock_->now();
  total_lidar_packets_received_++;
}

void SensorDiagnosticsState::record_imu_packet()
{
  last_successful_imu_frame_ = clock_->now();
  total_imu_packets_received_++;
}

void SensorDiagnosticsState::increment_poll_client_errors()
{
  last_client_error_ = clock_->now();
  poll_client_error_count_++;
}

void SensorDiagnosticsState::increment_lidar_packet_errors()
{
  last_unsuccessful_lidar_frame_ = clock_->now();
  read_lidar_packet_errors_++;
}

void SensorDiagnosticsState::increment_imu_packet_errors()
{
  last_unsuccessful_imu_frame_ = clock_->now();
  read_imu_packet_errors_++;
}

void SensorDiagnosticsState::notify_reset_sensor()
{
  last_sensor_reset_ = clock_->now();
  total_sensor_resets_++;
}

rclcpp::Time SensorDiagnosticsState::get_zero_time() const
{
  return rclcpp::Time(0, 0, clock_->get_clock_type());
}

void SensorDiagnosticsState::update_status(
  const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
  const std::map<std::string, std::string> & debug_context)
{
  current_message_ = message;
  current_level_ = level;
  current_debug_context_ = debug_context;
  message_history_.clear();
}

diagnostic_msgs::msg::DiagnosticStatus SensorDiagnosticsState::get_current_status() const
{
  return create_diagnostic_status(current_message_, current_level_, current_debug_context_);
}

void SensorDiagnosticsState::add_message_analysis(
  const std::string & topic_name, const std::vector<diagnostic_msgs::msg::KeyValue> & analysis)
{
  message_history_[topic_name].push_back(analysis);
}

std::map<std::string, std::string> SensorDiagnosticsState::get_debug_context(
  const std::string & sensor_hostname) const
{
  std::map<std::string, std::string> context;

  context["Sensor Hostname"] = sensor_hostname;

  auto now = clock_->now();
  if (last_successful_lidar_frame_.nanoseconds() > 0) {
    auto time_since_lidar_duration = now - last_successful_lidar_frame_;
    auto time_since_lidar_ms = time_since_lidar_duration.nanoseconds() / 1000000;
    context["Time Since Last LiDAR Frame (ms)"] = std::to_string(time_since_lidar_ms);
  } else {
    context["Time Since Last LiDAR Frame (ms)"] = "Never received";
  }

  if (last_successful_imu_frame_.nanoseconds() > 0) {
    auto time_since_imu_duration = now - last_successful_imu_frame_;
    auto time_since_imu_ms = time_since_imu_duration.nanoseconds() / 1000000;
    context["Time Since Last IMU Frame (ms)"] = std::to_string(time_since_imu_ms);
  } else {
    context["Time Since Last IMU Frame (ms)"] = "Never received";
  }

  return context;
}

diagnostic_msgs::msg::DiagnosticStatus SensorDiagnosticsState::create_diagnostic_status(
  const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
  const std::map<std::string, std::string> & debug_context) const
{
  diagnostic_msgs::msg::DiagnosticStatus status;

  status.hardware_id = hardware_id_;
  status.message = message;
  status.level = level;
  status.name = name_;

  auto now = clock_->now();

  auto add_kv = [](
                  std::vector<diagnostic_msgs::msg::KeyValue> & values, const std::string & key,
                  const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    values.push_back(kv);
  };

  add_kv(status.values, "Last Update", std::to_string(now.seconds()));
  add_kv(status.values, "Last Sensor Reset", std::to_string(last_sensor_reset_.seconds()));
  add_kv(status.values, "Total Sensor Resets", std::to_string(total_sensor_resets_));
  add_kv(status.values, "Sensor Uptime (s)", std::to_string((now - sensor_start_time_).seconds()));

  add_kv(
    status.values, "Last successful LiDAR frame",
    std::to_string(last_successful_lidar_frame_.seconds()));
  add_kv(
    status.values, "Last unsuccessful LiDAR frame",
    std::to_string(last_unsuccessful_lidar_frame_.seconds()));
  add_kv(status.values, "Total LIDAR Packets", std::to_string(total_lidar_packets_received_));
  add_kv(status.values, "LIDAR Packet Errors", std::to_string(read_lidar_packet_errors_));

  add_kv(
    status.values, "Last successful IMU frame",
    std::to_string(last_successful_imu_frame_.seconds()));
  add_kv(
    status.values, "Last unsuccessful IMU frame",
    std::to_string(last_unsuccessful_imu_frame_.seconds()));
  add_kv(status.values, "Total IMU Packets", std::to_string(total_imu_packets_received_));
  add_kv(status.values, "IMU Packet Errors", std::to_string(read_imu_packet_errors_));

  add_kv(status.values, "Poll Client Errors", std::to_string(poll_client_error_count_));
  add_kv(status.values, "Last Client Error", std::to_string(last_client_error_.seconds()));

  for (const auto & metadata_pair : sensor_info_) {
    add_kv(status.values, "Sensor " + metadata_pair.first, metadata_pair.second);
  }

  for (const auto & context_pair : debug_context) {
    add_kv(status.values, context_pair.first, context_pair.second);
  }

  std::vector<diagnostic_msgs::msg::KeyValue> message_history_kv;
  for (const auto & [topic_name, analyses] : message_history_) {
    for (const auto & analysis : analyses) {
      for (const auto & kv : analysis) {
        add_kv(message_history_kv, topic_name + " " + kv.key, kv.value);
      }
    }
  }

  // Sort and unique keys to avoid duplicates, leave the most recent ones
  std::stable_sort(
    message_history_kv.rbegin(), message_history_kv.rend(),
    [](const diagnostic_msgs::msg::KeyValue & a, const diagnostic_msgs::msg::KeyValue & b) {
      return a.key < b.key;
    });

  auto unique_end = std::unique(
    message_history_kv.rbegin(), message_history_kv.rend(),
    [](const diagnostic_msgs::msg::KeyValue & a, const diagnostic_msgs::msg::KeyValue & b) {
      return a.key == b.key;
    });

  std::move(message_history_kv.rbegin(), unique_end, std::back_inserter(status.values));

  return status;
}

void SensorDiagnosticsState::update_metadata(const ouster::sensor::sensor_info & info)
{
  sensor_info_.clear();

  sensor_info_["Serial Number"] = info.sn;
  sensor_info_["Firmware Version"] = info.fw_rev;
  sensor_info_["Product Line"] = info.prod_line;
  sensor_info_["Product Part Number"] = info.prod_pn;

  sensor_info_["Lidar Mode"] = ouster::sensor::to_string(info.mode);
  sensor_info_["Beam Azimuth Angles"] = std::to_string(info.beam_azimuth_angles.size());
  sensor_info_["Beam Altitude Angles"] = std::to_string(info.beam_altitude_angles.size());
  sensor_info_["Pixels Per Column"] = std::to_string(info.format.pixels_per_column);
  sensor_info_["Columns Per Packet"] = std::to_string(info.format.columns_per_packet);
  sensor_info_["Columns Per Frame"] = std::to_string(info.format.columns_per_frame);
  sensor_info_["Info JSON Length"] = std::to_string(info.original_string().length());
}

}  // namespace ouster_ros
