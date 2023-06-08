/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.h
 * @brief Base class for ouster_ros os_cloud and os_image nodes
 *
 */

#include <ouster/types.h>

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace ouster_ros {

class OusterProcessingNodeBase : public rclcpp::Node {
   protected:
    OusterProcessingNodeBase(const std::string& name,
                             const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options) {}

    void create_metadata_subscriber(
        std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>
            on_sensor_metadata);

   protected:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub;
    ouster::sensor::sensor_info info;
};

}  // namespace ouster_ros