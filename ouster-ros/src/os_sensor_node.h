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
#include <vector>

#include <std_srvs/srv/empty.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include "ouster_sensor_msgs/msg/packet_msg.hpp"
#include "ouster_sensor_msgs/srv/get_config.hpp"
#include "ouster_sensor_msgs/srv/set_config.hpp"
#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_sensor_node_base.h"

#include "thread_safe_ring_buffer.h"

namespace sensor = ouster::sensor;
using lifecycle_msgs::srv::ChangeState;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace ouster_ros {

class OusterSensor : public OusterSensorNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    OusterSensor(const std::string& name, const rclcpp::NodeOptions& options);
    explicit OusterSensor(const rclcpp::NodeOptions& options);
    ~OusterSensor() override;

    LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& state);
    LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& state);
    LifecycleNodeInterface::CallbackReturn on_error(
        const rclcpp_lifecycle::State& state);
    LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& state);
    LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State& state);
    LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state);

   protected:
    virtual void on_metadata_updated(const sensor::sensor_info& info);

    virtual void create_services();

    virtual void create_publishers();

    virtual void on_lidar_packet_msg(const uint8_t* raw_lidar_packet);

    virtual void on_imu_packet_msg(const uint8_t* raw_imu_packet);

    virtual void cleanup();

    void halt();

   private:
    void declare_parameters();

    std::string get_sensor_hostname();

    void update_config_and_metadata(sensor::client& client);

    void save_metadata();

    static std::string transition_id_to_string(uint8_t transition_id);
    template <typename CallbackT, typename... CallbackT_Args>
    bool change_state(std::uint8_t transition_id, CallbackT callback,
                      CallbackT_Args... callback_args,
                      std::chrono::seconds time_out = std::chrono::seconds{3});

    void execute_transitions_sequence(std::vector<uint8_t> transitions_sequence,
                                      size_t at);

    // param init_id_reset is overriden to true when force_reinit is true
    void reset_sensor(bool force_reinit, bool init_id_reset = false);

    // TODO: need to notify dependent node(s) of the update
    void reactivate_sensor(bool init_id_reset = false);

    void create_reset_service();

    void create_get_config_service();

    void create_set_config_service();

    std::shared_ptr<sensor::client> create_sensor_client(
        const std::string& hostname, const sensor::sensor_config& config);

    sensor::sensor_config parse_config_from_ros_parameters();

    sensor::sensor_config parse_config_from_staged_config_string();

    uint8_t compose_config_flags(const sensor::sensor_config& config);

    void configure_sensor(const std::string& hostname,
                          sensor::sensor_config& config);

    std::string load_config_file(const std::string& config_file);

    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode);

    void allocate_buffers();

    bool init_id_changed(const sensor::packet_format& pf,
                         const uint8_t* lidar_buf);

    void handle_poll_client_error();

    void handle_lidar_packet(sensor::client& client,
                             const sensor::packet_format& pf);

    void handle_imu_packet(sensor::client& client,
                           const sensor::packet_format& pf);

    void connection_loop(sensor::client& client,
                         const sensor::packet_format& pf);

    void start_sensor_connection_thread();

    void stop_sensor_connection_thread();

    void start_packet_processing_threads();

    void stop_packet_processing_threads();

   private:
    std::string sensor_hostname;
    std::string staged_config;
    std::string active_config;
    std::string mtp_dest;
    bool mtp_main;
    std::shared_ptr<sensor::client> sensor_client;
    ouster_sensor_msgs::msg::PacketMsg lidar_packet;
    ouster_sensor_msgs::msg::PacketMsg imu_packet;
    rclcpp::Publisher<ouster_sensor_msgs::msg::PacketMsg>::SharedPtr lidar_packet_pub;
    rclcpp::Publisher<ouster_sensor_msgs::msg::PacketMsg>::SharedPtr imu_packet_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
    rclcpp::Service<ouster_sensor_msgs::srv::GetConfig>::SharedPtr get_config_srv;
    rclcpp::Service<ouster_sensor_msgs::srv::SetConfig>::SharedPtr set_config_srv;
    std::shared_ptr<rclcpp::Client<ChangeState>> change_state_client;

    // TODO: implement & utilize a lock-free ring buffer in future
    std::unique_ptr<ThreadSafeRingBuffer> lidar_packets;
    std::unique_ptr<ThreadSafeRingBuffer> imu_packets;

    std::atomic<bool> sensor_connection_active = {false};
    std::unique_ptr<std::thread> sensor_connection_thread;

    std::atomic<bool> imu_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> imu_packets_processing_thread;

    std::atomic<bool> lidar_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> lidar_packets_processing_thread;

    bool force_sensor_reinit = false;
    bool reset_last_init_id = true;

    bool last_init_id_initialized = false;
    uint32_t last_init_id;

    // TODO: add as a ros parameter
    const int max_poll_client_error_count = 10;
    int poll_client_error_count = 0;
    // TODO: add as a ros parameter
    const int max_read_lidar_packet_errors = 60;
    int read_lidar_packet_errors = 0;
    // TODO: add as a ros parameter
    const int max_read_imu_packet_errors = 60;
    int read_imu_packet_errors = 0;
};

}  // namespace ouster_ros