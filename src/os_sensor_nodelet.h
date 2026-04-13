/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node.h
 * @brief A node that connects to a live ouster sensor
 */

#pragma once

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <string>
#include <thread>
#include <atomic>
#include <optional>

#include <ouster/client.h>

#include "ouster_ros/GetConfig.h"
#include "ouster_ros/SetConfig.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os_sensor_nodelet_base.h"


namespace ouster_ros {

class OusterSensor : public OusterSensorNodeletBase {
   public:
    ~OusterSensor() override;

   protected:
    virtual void onInit() override;

    virtual void on_metadata_updated(const ouster::sdk::core::SensorInfo& info);

    virtual void create_services();

    virtual void create_publishers();

    virtual void on_lidar_packet_msg(const ouster::sdk::core::LidarPacket& lidar_packet);

    virtual void on_imu_packet_msg(const ouster::sdk::core::ImuPacket& imu_packet);

    bool start();

    void stop();

    void attempt_start();

    void schedule_stop();

    void start_sensor_connection_thread();

    void stop_sensor_connection_thread();

   private:
    std::string get_sensor_hostname();

    void update_metadata(ouster::sdk::sensor::Client& cli);

    void metadata_updated(const ouster::sdk::core::SensorInfo& info);

    void save_metadata();

    // param init_id_reset is overriden to true when force_reinit is true
    void reset_sensor(bool force_reinit, bool init_id_reset = false);

    void reactivate_sensor(bool init_id_reset = false);

    void create_reset_service();

    void create_get_config_service();

    void create_set_config_service();

    std::shared_ptr<ouster::sdk::sensor::Client> create_sensor_client(
        const std::string& hostname, const ouster::sdk::core::SensorConfig& config);

    ouster::sdk::core::SensorConfig parse_config_from_ros_parameters();

// helper methods for parsing individual config fields from ros parameters
    void parse_udp_dest_and_ports(ouster::sdk::core::SensorConfig& config);
    void parse_udp_profile_lidar(ouster::sdk::core::SensorConfig& config);
    void parse_udp_profile_imu_and_settings(ouster::sdk::core::SensorConfig& config);
    void parse_lidar_mode(ouster::sdk::core::SensorConfig& config);
    void parse_timestamp_mode(ouster::sdk::core::SensorConfig& config);
    void parse_azimuth_window(ouster::sdk::core::SensorConfig& config);
    void parse_operating_mode(ouster::sdk::core::SensorConfig& config);
    void parse_signal_multiplier(ouster::sdk::core::SensorConfig& config);
    void parse_phase_lock_and_offset(ouster::sdk::core::SensorConfig& config);
    void parse_min_distance(ouster::sdk::core::SensorConfig& config);
    void parse_multipurpose_io_mode(ouster::sdk::core::SensorConfig& config);
    void parse_nmea_in_polarity(ouster::sdk::core::SensorConfig& config);
    void parse_nmea_ignore_valid_char(ouster::sdk::core::SensorConfig& config);
    void parse_nmea_baud_rate(ouster::sdk::core::SensorConfig& config);
    void parse_nmea_leap_seconds(ouster::sdk::core::SensorConfig& config);
    void parse_sync_pulse_in_polarity(ouster::sdk::core::SensorConfig& config);
    void parse_sync_pulse_out_polarity(ouster::sdk::core::SensorConfig& config);
    void parse_sync_pulse_out_frequency(ouster::sdk::core::SensorConfig& config);
    void parse_sync_pulse_out_angle(ouster::sdk::core::SensorConfig& config);
    void parse_sync_pulse_out_pulse_width(ouster::sdk::core::SensorConfig& config);
    void parse_lidar_frame_azimuth_offset(ouster::sdk::core::SensorConfig& config);
    void parse_bloom_reduction_optimization(ouster::sdk::core::SensorConfig& config);
    void parse_return_order(ouster::sdk::core::SensorConfig& config);
    void parse_persist_config_flag();

    uint8_t compose_config_flags(const ouster::sdk::core::SensorConfig& config);

    bool configure_sensor(const std::string& hostname,
                          ouster::sdk::core::SensorConfig& config);

    std::string load_config_file(const std::string& config_file);

    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(ouster::sdk::core::SensorInfo& info,
                                    ouster::sdk::core::LidarMode specified_lidar_mode);

    void allocate_buffers();

    bool init_id_changed(const ouster::sdk::core::PacketFormat& pf,
                         const ouster::sdk::core::LidarPacket& lidar_packet);

    void handle_poll_client_error();

    void read_lidar_packet(ouster::sdk::sensor::Client& client,
                           const ouster::sdk::core::PacketFormat& pf);

    void handle_lidar_packet(const ouster::sdk::core::LidarPacket& lidar_packet);

    void read_imu_packet(ouster::sdk::sensor::Client& client,
                         const ouster::sdk::core::PacketFormat& pf);

    void handle_imu_packet(const ouster::sdk::core::ImuPacket& imu_packet);

    void cleanup();

    void connection_loop(ouster::sdk::sensor::Client& client,
                         const ouster::sdk::core::PacketFormat& pf);

    bool get_active_config_no_throw(const std::string& sensor_hostname,
                                    ouster::sdk::core::SensorConfig& config);

   private:
    std::string sensor_hostname;
    std::optional<ouster::sdk::core::SensorConfig> staged_config;
    std::string mtp_dest;
    bool mtp_main;
    std::shared_ptr<ouster::sdk::sensor::Client> sensor_client;
    PacketMsg lidar_packet_msg;
    PacketMsg imu_packet_msg;
    ouster::sdk::core::LidarPacket lidar_packet;
    ouster::sdk::core::ImuPacket imu_packet;
    ros::Publisher lidar_packet_pub;
    ros::Publisher imu_packet_pub;
    ros::ServiceServer reset_srv;
    ros::ServiceServer get_config_srv;
    ros::ServiceServer set_config_srv;

    std::atomic<bool> sensor_connection_active = {false};
    std::unique_ptr<std::thread> sensor_connection_thread;

    std::atomic<bool> imu_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> imu_packets_processing_thread;

    std::atomic<bool> lidar_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> lidar_packets_processing_thread;

    bool persist_config = false;
    bool force_sensor_reinit = false;
    bool auto_udp_allowed = false;
    bool reset_last_init_id = true;
    std::optional<uint32_t> last_init_id;

    // TODO: add as a ros parameter
    const int max_poll_client_error_count = 10;
    int poll_client_error_count = 0;
    // TODO: add as a ros parameter
    const int max_read_lidar_packet_errors = 60;
    int read_lidar_packet_errors = 0;
    // TODO: add as a ros parameter
    const int max_read_imu_packet_errors = 60;
    int read_imu_packet_errors = 0;

    const int MIN_AZW = 0;
    const int MAX_AZW = 360000;

    bool attempt_reconnect;
    double dormant_period_between_reconnects;
    int reconnect_attempts_available;
    ros::Timer reconnect_timer;
};

}  // namespace ouster_ros
