/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_diagnostics_tracker.h
 * @brief Diagnostic tracking functionality for Ouster sensors using ROS2 diagnostic_updater with message analysis
 */

#pragma once

#include <ouster/types.h>

#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <map>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ouster_ros
{

class SensorDiagnosticsTracker final
{
public:
  SensorDiagnosticsTracker  (
    const std::string & name, rclcpp::Clock::SharedPtr clock, const std::string & hardware_id = "")
  : name_{name},
    hardware_id_{hardware_id.empty() ? name : hardware_id},
    clock_{clock},
    sensor_start_time_{clock_->now()}
  {
  }

  void record_lidar_packet();
  void record_imu_packet();
  void increment_poll_client_errors();
  void increment_lidar_packet_errors();
  void increment_imu_packet_errors();
  void notify_reset_sensor();
  void update_metadata(const ouster::sensor::sensor_info & info);

  std::map<std::string, std::string> get_debug_context(
    const std::string & sensor_hostname, bool sensor_connection_active) const;

  diagnostic_msgs::msg::DiagnosticStatus create_diagnostic_status(
    const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string> & debug_context = {}) const;

  void update_status(
    const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string> & debug_context = {});

  diagnostic_msgs::msg::DiagnosticStatus get_current_status() const;

  void add_message_analysis(
    const std::string & topic_name, const std::vector<diagnostic_msgs::msg::KeyValue> & analysis);

  const std::string & get_hardware_id() const { return hardware_id_; }
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }
  const rclcpp::Time & get_sensor_start_time() const { return sensor_start_time_; }
  uint32_t get_total_lidar_packets() const { return total_lidar_packets_received_; }
  uint32_t get_total_imu_packets() const { return total_imu_packets_received_; }
  uint32_t get_total_sensor_resets() const { return total_sensor_resets_; }
  uint32_t get_poll_client_error_count() const { return poll_client_error_count_; }
  uint32_t get_read_lidar_packet_errors() const { return read_lidar_packet_errors_; }
  uint32_t get_read_imu_packet_errors() const { return read_imu_packet_errors_; }
  const std::map<std::string, std::string> & get_sensor_info() const { return sensor_info_; }
  const rclcpp::Time & get_last_successful_lidar_frame() const
  {
    return last_successful_lidar_frame_;
  }
  const rclcpp::Time & get_last_successful_imu_frame() const { return last_successful_imu_frame_; }

protected:
  std::string name_;
  std::string hardware_id_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::Time sensor_start_time_;
  rclcpp::Time last_successful_lidar_frame_;
  rclcpp::Time last_successful_imu_frame_;
  rclcpp::Time last_unsuccessful_lidar_frame_;
  rclcpp::Time last_unsuccessful_imu_frame_;
  rclcpp::Time last_client_error_;
  rclcpp::Time last_sensor_reset_;

  uint32_t total_lidar_packets_received_{0};
  uint32_t total_imu_packets_received_{0};
  uint32_t total_sensor_resets_{0};

  uint32_t poll_client_error_count_{0};
  uint32_t read_lidar_packet_errors_{0};
  uint32_t read_imu_packet_errors_{0};
  std::map<std::string, std::string> sensor_info_;

  std::string current_message_{"Not initialized"};
  diagnostic_msgs::msg::DiagnosticStatus::_level_type current_level_{
    diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::map<std::string, std::string> current_debug_context_;

private:
  rclcpp::Time get_zero_time() const;
};

}  // namespace ouster_ros
