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

#include "ouster_ros/GetConfig.h"
#include "ouster_ros/SetConfig.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os_sensor_nodelet_base.h"

namespace sensor = ouster::sensor;

namespace ouster_ros {

class OusterSensor : public OusterSensorNodeletBase {
   public:
    ~OusterSensor() override;

   protected:
    virtual void onInit() override;

    virtual void on_metadata_updated(const sensor::sensor_info& info);

    virtual void create_services();

    virtual void create_publishers();

    virtual void on_lidar_packet_msg(const sensor::LidarPacket& lidar_packet);

    virtual void on_imu_packet_msg(const sensor::ImuPacket& imu_packet);

    bool start();

    void stop();

    void attempt_start();

    void schedule_stop();

    void start_sensor_connection_thread();

    void stop_sensor_connection_thread();

   private:
    std::string get_sensor_hostname();

    void update_metadata(sensor::client& client);

    void metadata_updated(const sensor::sensor_info& info);

    void save_metadata();

    // param init_id_reset is overriden to true when force_reinit is true
    void reset_sensor(bool force_reinit, bool init_id_reset = false);

    void reactivate_sensor(bool init_id_reset = false);

    void create_reset_service();

    void create_get_config_service();

    void create_set_config_service();

    std::shared_ptr<sensor::client> create_sensor_client(
        const std::string& hostname, const sensor::sensor_config& config);

    sensor::sensor_config parse_config_from_ros_parameters();

    uint8_t compose_config_flags(const sensor::sensor_config& config);

    bool configure_sensor(const std::string& hostname,
                          sensor::sensor_config& config);

    std::string load_config_file(const std::string& config_file);

    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode);

    void allocate_buffers();

    bool init_id_changed(const sensor::packet_format& pf,
                         const sensor::LidarPacket& lidar_packet);

    void handle_poll_client_error();

    void read_lidar_packet(sensor::client& client,
                           const sensor::packet_format& pf);

    void handle_lidar_packet(const sensor::LidarPacket& lidar_packet);

    void read_imu_packet(sensor::client& client,
                         const sensor::packet_format& pf);

    void handle_imu_packet(const sensor::ImuPacket& imu_packet);

    void cleanup();

    void connection_loop(sensor::client& client,
                         const sensor::packet_format& pf);

    bool get_active_config_no_throw(const std::string& sensor_hostname,
                                    sensor::sensor_config& config);

   private:
    std::string sensor_hostname;
    std::optional<sensor::sensor_config> staged_config;
    std::string mtp_dest;
    bool mtp_main;
    std::shared_ptr<sensor::client> sensor_client;
    PacketMsg lidar_packet_msg;
    PacketMsg imu_packet_msg;
    ouster::sensor::LidarPacket lidar_packet;
    ouster::sensor::ImuPacket imu_packet;
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

    bool attempt_reconnect;
    double dormant_period_between_reconnects;
    int reconnect_attempts_available;
    ros::Timer reconnect_timer;
};

}  // namespace ouster_ros
