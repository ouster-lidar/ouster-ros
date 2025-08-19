/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file sensor_diagnostics_tracker.cpp
 * @brief Implementation of diagnostic tracking functionality for Ouster sensors
 */

#include "ouster_ros/sensor_diagnostics_tracker.h"

namespace ouster_ros {

SensorDiagnosticsTracker::SensorDiagnosticsTracker(
    const std::string& name, rclcpp::Clock::SharedPtr clock,
    const std::string& hardware_id)
    : name_{name},
      hardware_id_{hardware_id.empty() ? name : hardware_id},
      clock_{clock},
      sensor_start_time_{clock_->now()},
      last_successful_lidar_frame_{rclcpp::Time(0)},
      last_successful_imu_frame_{rclcpp::Time(0)},
      total_lidar_packets_received_{0},
      total_imu_packets_received_{0},
      poll_client_error_count_{0},
      read_lidar_packet_errors_{0},
      read_imu_packet_errors_{0} {}

void SensorDiagnosticsTracker::record_lidar_packet() {
    last_successful_lidar_frame_ = clock_->now();
    total_lidar_packets_received_++;
}

void SensorDiagnosticsTracker::record_imu_packet() {
    last_successful_imu_frame_ = clock_->now();
    total_imu_packets_received_++;
}

void SensorDiagnosticsTracker::increment_poll_client_errors() {
    poll_client_error_count_++;
}

void SensorDiagnosticsTracker::increment_lidar_packet_errors() {
    read_lidar_packet_errors_++;
}

void SensorDiagnosticsTracker::increment_imu_packet_errors() {
    read_imu_packet_errors_++;
}

void SensorDiagnosticsTracker::reset_poll_client_errors() {
    poll_client_error_count_ = 0;
}

void SensorDiagnosticsTracker::reset_lidar_packet_errors() {
    read_lidar_packet_errors_ = 0;
}

void SensorDiagnosticsTracker::reset_imu_packet_errors() {
    read_imu_packet_errors_ = 0;
}

std::map<std::string, std::string> SensorDiagnosticsTracker::get_debug_context(
    const std::string& sensor_hostname, bool sensor_connection_active) const {
    std::map<std::string, std::string> context;

    auto now = clock_->now();

    context["Sensor Hostname"] = sensor_hostname;

    if (sensor_start_time_.nanoseconds() > 0) {
        auto uptime_duration = now - sensor_start_time_;
        auto uptime_ms =
            uptime_duration.nanoseconds() / 1000000;  // Convert to milliseconds
        context["Sensor Uptime (ms)"] = std::to_string(uptime_ms);
    }

    if (last_successful_lidar_frame_.nanoseconds() > 0) {
        auto time_since_lidar_duration = now - last_successful_lidar_frame_;
        auto time_since_lidar_ms =
            time_since_lidar_duration.nanoseconds() / 1000000;
        context["Time Since Last LiDAR Frame (ms)"] =
            std::to_string(time_since_lidar_ms);
    } else {
        context["Time Since Last LiDAR Frame (ms)"] = "Never received";
    }

    if (last_successful_imu_frame_.nanoseconds() > 0) {
        auto time_since_imu_duration = now - last_successful_imu_frame_;
        auto time_since_imu_ms =
            time_since_imu_duration.nanoseconds() / 1000000;
        context["Time Since Last IMU Frame (ms)"] =
            std::to_string(time_since_imu_ms);
    } else {
        context["Time Since Last IMU Frame (ms)"] = "Never received";
    }

    context["Sensor Connection Active"] =
        sensor_connection_active ? "true" : "false";

    return context;
}

diagnostic_msgs::msg::DiagnosticStatus
SensorDiagnosticsTracker::create_diagnostic_status(
    const std::string& message,
    diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string>& debug_context) const {
    diagnostic_msgs::msg::DiagnosticStatus status;

    status.hardware_id = hardware_id_;
    status.message = message;
    status.level = level;
    status.name = name_;

    status.values.clear();

    auto now = clock_->now();

    // Helper function to add a key-value pair to status.values
    auto add_kv = [](std::vector<diagnostic_msgs::msg::KeyValue>& values,
                     const std::string& key, const std::string& value) {
        diagnostic_msgs::msg::KeyValue kv;
        kv.key = key;
        kv.value = value;
        values.push_back(kv);
    };

    add_kv(status.values, "Last Update", std::to_string(now.seconds()));
    add_kv(status.values, "Sensor Uptime (s)",
           std::to_string((now - sensor_start_time_).seconds()));
    add_kv(status.values, "Last successful LiDAR frame",
           std::to_string(last_successful_lidar_frame_.seconds()));
    add_kv(status.values, "Last successful IMU frame",
           std::to_string(last_successful_imu_frame_.seconds()));
    add_kv(status.values, "Total LIDAR Packets",
           std::to_string(total_lidar_packets_received_));
    add_kv(status.values, "Total IMU Packets",
           std::to_string(total_imu_packets_received_));
    add_kv(status.values, "Poll Client Error Count",
           std::to_string(poll_client_error_count_));
    add_kv(status.values, "LIDAR Packet Error Count",
           std::to_string(read_lidar_packet_errors_));
    add_kv(status.values, "IMU Packet Error Count",
           std::to_string(read_imu_packet_errors_));

    // Add sensor metadata if available
    for (const auto& metadata_pair : sensor_info_) {
        add_kv(status.values, "Sensor " + metadata_pair.first,
               metadata_pair.second);
    }

    for (const auto& context_pair : debug_context) {
        add_kv(status.values, context_pair.first, context_pair.second);
    }

    return status;
}

void SensorDiagnosticsTracker::update_metadata(
    const ouster::sensor::sensor_info& info) {
    sensor_info_.clear();

    // Basic sensor information
    sensor_info_["Product Line"] = info.prod_line;
    sensor_info_["Serial Number"] = info.sn;
    sensor_info_["Firmware Version"] = info.fw_rev;

    // Lidar configuration
    sensor_info_["Lidar Mode"] = ouster::sensor::to_string(info.mode);
    sensor_info_["UDP Profile Lidar"] =
        ouster::sensor::to_string(info.format.udp_profile_lidar);
    sensor_info_["UDP Profile IMU"] =
        ouster::sensor::to_string(info.format.udp_profile_imu);

    // Network configuration (handling optional values)
    if (info.config.udp_port_lidar.has_value()) {
        sensor_info_["UDP Port Lidar"] =
            std::to_string(info.config.udp_port_lidar.value());
    } else {
        sensor_info_["UDP Port Lidar"] = "Not set";
    }

    if (info.config.udp_port_imu.has_value()) {
        sensor_info_["UDP Port IMU"] =
            std::to_string(info.config.udp_port_imu.value());
    } else {
        sensor_info_["UDP Port IMU"] = "Not set";
    }

    // Convert UDP destination to string
    if (info.config.udp_dest.has_value()) {
        sensor_info_["UDP Dest"] = info.config.udp_dest.value();
    } else {
        sensor_info_["UDP Dest"] = "Not set";
    }

    // Timing and operational settings
    sensor_info_["Columns Per Packet"] =
        std::to_string(info.format.columns_per_packet);

    // Column window information
    if (info.format.column_window.first != info.format.column_window.second) {
        sensor_info_["Column Window"] =
            "[" + std::to_string(info.format.column_window.first) + ", " +
            std::to_string(info.format.column_window.second) + "]";
    }

    // Sensor capabilities and status
    sensor_info_["Lidar Data Format"] =
        "Columns: " + std::to_string(info.format.columns_per_frame) +
        ", Pixels: " + std::to_string(info.format.pixels_per_column);

    // Beam configuration (for diagnostic purposes)
    if (!info.beam_azimuth_angles.empty() &&
        !info.beam_altitude_angles.empty()) {
        sensor_info_["Beam Count"] =
            std::to_string(info.beam_azimuth_angles.size());

        auto alt_min = *std::min_element(info.beam_altitude_angles.begin(),
                                         info.beam_altitude_angles.end());
        auto alt_max = *std::max_element(info.beam_altitude_angles.begin(),
                                         info.beam_altitude_angles.end());
        sensor_info_["Altitude Range"] = "[" + std::to_string(alt_min) + ", " +
                                         std::to_string(alt_max) + "] degrees";
    }

    // Additional useful diagnostic information
    sensor_info_["Sensor Info JSON Length"] =
        std::to_string(info.original_string().length());
}

void SensorDiagnosticsTracker::update_status(
    const std::string& message,
    diagnostic_msgs::msg::DiagnosticStatus::_level_type level,
    const std::map<std::string, std::string>& debug_context) {
    current_message_ = message;
    current_level_ = level;
    current_debug_context_ = debug_context;
}

diagnostic_msgs::msg::DiagnosticStatus
SensorDiagnosticsTracker::get_current_status() const {
    return create_diagnostic_status(current_message_, current_level_,
                                    current_debug_context_);
}

}  // namespace ouster_ros
