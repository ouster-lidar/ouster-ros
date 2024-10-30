/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node_base.h
 * @brief Base class for ouster_ros sensor and replay nodes
 *
 */

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ouster_sensor_msgs/srv/get_metadata.hpp"

#include <ouster/types.h>

namespace ouster_ros {

class OusterSensorNodeBase : public rclcpp_lifecycle::LifecycleNode {
   protected:
    explicit OusterSensorNodeBase(const std::string& name,
                                  const rclcpp::NodeOptions& options);

   protected:
    bool is_arg_set(const std::string& arg) const {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void create_get_metadata_service();

    void create_metadata_pub();

    void publish_metadata();

    void display_lidar_info(const ouster::sensor::sensor_info& info);

    static std::string read_text_file(const std::string& text_file);

    static bool write_text_to_file(const std::string& file_path,
                                   const std::string& text);

    static std::string transition_id_to_string(uint8_t transition_id);

    template <typename CallbackT, typename... CallbackT_Args>
    bool change_state(std::uint8_t transition_id, CallbackT callback,
                      CallbackT_Args... callback_args,
                      std::chrono::seconds time_out = std::chrono::seconds{3});

    void execute_transitions_sequence(std::vector<uint8_t> transitions_sequence,
                                      size_t at);


   protected:
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> change_state_client;

    ouster::sensor::sensor_info info;
    rclcpp::Service<ouster_sensor_msgs::srv::GetMetadata>::SharedPtr get_metadata_srv;
    std::string cached_metadata;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub;
};

}  // namespace ouster_ros