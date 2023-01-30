/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_node.h
 * @brief Base class for ouster_ros sensor and replay nodes
 *
 */

#include <rclcpp/rclcpp.hpp>

#include <ouster/types.h>

#include "ouster_srvs/srv/get_metadata.hpp"

namespace ouster_ros {

class OusterClientBase : public rclcpp::Node {
   protected:
    explicit OusterClientBase(const std::string& name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(name, options) {
    }

    virtual void onInit();

   protected:
    bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void display_lidar_info(const ouster::sensor::sensor_info& info);

   protected:
    ouster::sensor::sensor_info info;
    rclcpp::Service<ouster_srvs::srv::GetMetadata>::SharedPtr get_metadata_srv;
    std::string cached_metadata;
};

}  // namespace ouster_ros