/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_diagnostics_tracker.h
 * @brief Diagnostic tracking functionality for Ouster sensors using ROS2 diagnostic_updater with message analysis
 */

#pragma once

#include <ouster/types.h>
#include <ouster_ros/diagnostics_visitor_registry.h>
#include <ouster_ros/ring_buffer.h>

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

class SensorDiagnosticsState final
{
public:
  SensorDiagnosticsState(
    const std::string & name, rclcpp::Clock::SharedPtr clock, const std::string & hardware_id = "")
  : name_{name},
    hardware_id_{hardware_id.empty() ? name : hardware_id},
    clock_{clock},
    sensor_start_time_{clock_->now()}
  {
  }

  ~SensorDiagnosticsState() = default;

  void record_lidar_packet();
  void record_imu_packet();
  void increment_poll_client_errors();
  void increment_lidar_packet_errors();
  void increment_imu_packet_errors();
  void notify_reset_sensor();
  void update_metadata(const ouster::sensor::sensor_info & info);

  std::map<std::string, std::string> get_debug_context(const std::string & sensor_hostname) const;

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

  std::map<std::string, RingBuffer<std::vector<diagnostic_msgs::msg::KeyValue>>> message_history_;

  std::string current_message_{"Not initialized"};
  diagnostic_msgs::msg::DiagnosticStatus::_level_type current_level_{
    diagnostic_msgs::msg::DiagnosticStatus::STALE};
  std::map<std::string, std::string> current_debug_context_;

private:
  rclcpp::Time get_zero_time() const;
};

template <typename DiagnosticsVisitorRegistryType>
class SensorDiagnosticsTracker
{
public:
  using DiagnosticsVisitorRegistryT = DiagnosticsVisitorRegistryType;

  template <typename NodeT>
  SensorDiagnosticsTracker(
    const std::string & name, NodeT * node, const std::string & hardware_id = "",
    const DiagnosticsVisitorRegistryType & msg_analyzer = DiagnosticsVisitorRegistryType{});

  SensorDiagnosticsTracker(
    const std::string & name, rclcpp::Clock::SharedPtr clock, const std::string & hardware_id = "",
    const DiagnosticsVisitorRegistryType & msg_analyzer = DiagnosticsVisitorRegistryType{});

  ~SensorDiagnosticsTracker() = default;

  void record_lidar_packet() { base_.record_lidar_packet(); }
  void record_imu_packet() { base_.record_imu_packet(); }
  void increment_poll_client_errors() { base_.increment_poll_client_errors(); }
  void increment_lidar_packet_errors() { base_.increment_lidar_packet_errors(); }
  void increment_imu_packet_errors() { base_.increment_imu_packet_errors(); }
  void notify_reset_sensor() { base_.notify_reset_sensor(); }
  void update_metadata(const ouster::sensor::sensor_info & info) { base_.update_metadata(info); }

  void force_update();

  void update_status(
    const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string> & debug_context = {});

  diagnostic_msgs::msg::DiagnosticStatus get_current_status() const;

  std::map<std::string, std::string> get_debug_context(const std::string & sensor_hostname) const;

  diagnostic_msgs::msg::DiagnosticStatus create_diagnostic_status(
    const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string> & debug_context = {}) const;

  template <typename MsgT>
  void record_msg(const std::string & topic_name, const MsgT & msgs);

private:
  void add_message_analysis(
    const std::string & topic_name, const std::vector<diagnostic_msgs::msg::KeyValue> & analysis);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  double get_packet_rate(uint32_t packet_count, const rclcpp::Time & start_time) const;
  bool is_sensor_healthy() const;

  rclcpp::Logger logger_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  // Current sensor status
  std::string sensor_hostname_{"unknown"};
  bool sensor_connection_active_{false};

  SensorDiagnosticsState base_;
  DiagnosticsVisitorRegistryType msg_analyzer_;
};

// Template implementation for ROS nodes
template <typename DiagnosticsVisitorRegistryType>
template <typename NodeT>
SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::SensorDiagnosticsTracker(
  const std::string & name, NodeT * node, const std::string & hardware_id,
  const DiagnosticsVisitorRegistryType & msg_analyzer)
: logger_(node->get_logger()),
  updater_(nullptr),
  base_(name, node->get_clock(), hardware_id),
  msg_analyzer_(msg_analyzer)
{
  updater_ = std::make_unique<diagnostic_updater::Updater>(node);
  updater_->setHardwareID(base_.get_hardware_id());
  updater_->add(name + " Status", this, &SensorDiagnosticsTracker::produce_diagnostics);
  RCLCPP_INFO(logger_, "Diagnostic updater initialized for %s", name.c_str());
}

// Standalone constructor for testing
template <typename DiagnosticsVisitorRegistryType>
SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::SensorDiagnosticsTracker(
  const std::string & name, rclcpp::Clock::SharedPtr clock, const std::string & hardware_id,
  const DiagnosticsVisitorRegistryType & msg_analyzer)
: logger_(rclcpp::get_logger("sensor_diagnostics_tracker")),
  updater_(nullptr),
  base_(name, clock, hardware_id),
  msg_analyzer_(msg_analyzer)
{
  RCLCPP_INFO(logger_, "Standalone sensor diagnostics tracker initialized for %s", name.c_str());
}

template <typename DiagnosticsVisitorRegistryType>
void SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::force_update()
{
  if (updater_) {
    updater_->force_update();
  }
}

template <typename DiagnosticsVisitorRegistryType>
void SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::update_status(
  const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
  const std::map<std::string, std::string> & debug_context)
{
  base_.update_status(message, level, debug_context);
}

template <typename DiagnosticsVisitorRegistryType>
diagnostic_msgs::msg::DiagnosticStatus
SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::get_current_status() const
{
  return base_.get_current_status();
}

template <typename DiagnosticsVisitorRegistryType>
std::map<std::string, std::string>
SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::get_debug_context(
  const std::string & sensor_hostname) const
{
  return base_.get_debug_context(sensor_hostname);
}

template <typename DiagnosticsVisitorRegistryType>
diagnostic_msgs::msg::DiagnosticStatus
SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::create_diagnostic_status(
  const std::string & message, diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
  const std::map<std::string, std::string> & debug_context) const
{
  return base_.create_diagnostic_status(message, level, debug_context);
}

template <typename DiagnosticsVisitorRegistryType>
template <typename MsgT>
void SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::record_msg(
  const std::string & topic_name, const MsgT & msgs)
{
  auto analysis = msg_analyzer_(msgs);
  base_.add_message_analysis(topic_name, analysis);
}

template <typename DiagnosticsVisitorRegistryType>
void SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  bool is_healthy = is_sensor_healthy();

  if (is_healthy) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Sensor operating normally");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Sensor health issues detected");
  }

  const auto & current_status = base_.get_current_status();
  auto diag_status = create_diagnostic_status(
    current_status.message, current_status.level,
    base_.get_debug_context(sensor_hostname_));
  std::move(diag_status.values.begin(), diag_status.values.end(), std::back_inserter(stat.values));
}

template <typename DiagnosticsVisitorRegistryType>
bool SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::is_sensor_healthy() const
{
  auto now = base_.get_clock()->now();

  const auto timeout_threshold = rclcpp::Duration::from_seconds(5.0);

  bool lidar_healthy = (now - base_.get_last_successful_lidar_frame()) < timeout_threshold;
  bool imu_healthy = (now - base_.get_last_successful_imu_frame()) < timeout_threshold;

  return (lidar_healthy && imu_healthy);
}

template <typename DiagnosticsVisitorRegistryType>
double SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>::get_packet_rate(
  uint32_t packet_count, const rclcpp::Time & start_time) const
{
  auto now = base_.get_clock()->now();
  auto duration = now - start_time;
  if (duration.seconds() > 0) {
    return static_cast<double>(packet_count) / duration.seconds();
  }
  return 0.0;
}

// Helper function to create trackers with message analyzers
template <typename DiagnosticsVisitorRegistryType>
auto make_sensor_diagnostics_tracker(
  const std::string & name, rclcpp::Node * node, const std::string & hardware_id,
  const DiagnosticsVisitorRegistryType & msg_analyzer)
{
  return SensorDiagnosticsTracker<DiagnosticsVisitorRegistryType>(
    name, node, hardware_id, msg_analyzer);
}

}  // namespace ouster_ros
