/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_diagnostics_tracker.h
 * @brief Diagnostic tracking functionality for Ouster sensors
 */

#pragma once

#include <ouster/types.h>

#include <cstdint>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ouster_ros {

class SensorDiagnosticsTracker {
   public:
    SensorDiagnosticsTracker(const std::string& name,
                             rclcpp::Clock::SharedPtr clock,
                             const std::string& hardware_id = "");
    ~SensorDiagnosticsTracker() = default;

    void record_lidar_packet();

    void record_imu_packet();

    void increment_poll_client_errors();

    void increment_lidar_packet_errors();

    void increment_imu_packet_errors();

    void reset_poll_client_errors();

    void reset_lidar_packet_errors();

    void reset_imu_packet_errors();

    std::map<std::string, std::string> get_debug_context(
        const std::string& sensor_hostname,
        bool sensor_connection_active) const;

    diagnostic_msgs::msg::DiagnosticStatus create_diagnostic_status(
        const std::string& message,
        diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
        const std::map<std::string, std::string>& debug_context = {}) const;

    void update_status(
        const std::string& message,
        diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
        const std::map<std::string, std::string>& debug_context = {});

    void update_metadata(const ouster::sensor::sensor_info& info);

    diagnostic_msgs::msg::DiagnosticStatus get_current_status() const;

   private:
    std::string name_;
    std::string hardware_id_;

    rclcpp::Clock::SharedPtr clock_;

    rclcpp::Time sensor_start_time_;
    rclcpp::Time last_successful_lidar_frame_;
    rclcpp::Time last_successful_imu_frame_;

    uint32_t total_lidar_packets_received_{0};
    uint32_t total_imu_packets_received_{0};

    uint32_t poll_client_error_count_{0};
    uint32_t read_lidar_packet_errors_{0};
    uint32_t read_imu_packet_errors_{0};
    std::map<std::string, std::string> sensor_info_;

    mutable std::string current_message_{"Not initialized"};
    mutable diagnostic_msgs::msg::DiagnosticStatus::_level_type current_level_{
        diagnostic_msgs::msg::DiagnosticStatus::STALE};
    mutable std::map<std::string, std::string> current_debug_context_;
};

}  // namespace ouster_ros
