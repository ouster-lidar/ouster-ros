/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node_base.h
 * @brief Base class for ouster_ros sensor and replay nodes
 *
 */

#include <ouster/types.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include "ouster_sensor_msgs/srv/get_metadata.hpp"

namespace ouster_ros {

class OusterSensorNodeBase : public rclcpp_lifecycle::LifecycleNode {
   protected:
    explicit OusterSensorNodeBase(const std::string& name,
                                  const rclcpp::NodeOptions& options)
        : rclcpp_lifecycle::LifecycleNode(name, options) {}

   protected:
    bool is_arg_set(const std::string& arg) const {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void create_get_metadata_service();

    void create_metadata_publisher();

    void publish_metadata();

    void display_lidar_info(const ouster::sensor::sensor_info& info);

    static std::string read_text_file(const std::string& text_file);

    static bool write_text_to_file(const std::string& file_path,
                                   const std::string& text);

   protected:
    ouster::sensor::sensor_info info;
    rclcpp::Service<ouster_sensor_msgs::srv::GetMetadata>::SharedPtr get_metadata_srv;
    std::string cached_metadata;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr metadata_pub;
};

}  // namespace ouster_ros