/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.h
 * @brief Base class for ouster_ros os_cloud and os_image nodes
 *
 */

#include <ouster/types.h>

#include <rclcpp/rclcpp.hpp>

#include "ouster_srvs/srv/get_metadata.hpp"

namespace ouster_ros {

class OusterProcessingNodeBase : public rclcpp::Node {
   protected:
    explicit OusterProcessingNodeBase(const std::string& name,
                                      const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options) {}

   protected:
    bool wait_for_get_metadata_service(
        std::shared_ptr<rclcpp::Client<ouster_srvs::srv::GetMetadata>> client);

    std::string get_metadata();

    int get_n_returns();

    std::string topic_for_return(std::string base, int idx) {
        if (idx == 0) return base;
        return base + std::to_string(idx + 1);
    }

   protected:
    ouster::sensor::sensor_info info;
};

}  // namespace ouster_ros