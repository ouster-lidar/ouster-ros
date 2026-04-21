/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_client_base_nodelet.h
 * @brief Base class for ouster_ros sensor and replay nodelets
 *
 */
#pragma once

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <ouster/types.h>

namespace ouster_ros {

class OusterSensorNodeletBase : public nodelet::Nodelet {
   protected:
    bool is_arg_set(const std::string& arg) const {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    void create_get_metadata_service();

    void create_metadata_pub();

    void publish_metadata();

    void display_lidar_info(const ouster::sdk::core::SensorInfo& info);

   protected:
    ouster::sdk::core::SensorInfo info;
    ros::ServiceServer get_metadata_srv;
    std::string cached_metadata;
    ros::Publisher metadata_pub;
};

}  // namespace ouster_ros