/**
 * Copyright (c) 2018-2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.h
 * @brief Base class for ouster_ros sensor and replay nodelets
 *
 */

#include <rclcpp/rclcpp.hpp>

#include <ouster/types.h>

#include "ouster_ros/srv/get_metadata.hpp"

namespace nodelets_os {

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
    rclcpp::Service<ouster_ros::srv::GetMetadata>::SharedPtr get_metadata_srv;
    std::string cached_metadata;
};

}  // namespace nodelets_os