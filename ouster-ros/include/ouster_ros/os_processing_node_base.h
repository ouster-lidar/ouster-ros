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

#include "ouster_srvs/srv/get_metadata.hpp"

namespace ouster_ros {

class OusterProcessingNodeBase : public rclcpp::Node {
   protected:
    explicit OusterProcessingNodeBase(const std::string& name,
                                      const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options) {}

   protected:
    bool spin_till_attempts_exahused(const std::string& log_msg,
                                     std::function<bool(void)> lambda);

    std::string get_metadata();

    int get_n_returns();

    std::string topic_for_return(std::string base, int idx) {
        if (idx == 0) return base;
        return base + std::to_string(idx + 1);
    }

   protected:
    // TODO: Add as node parameters?
    static const std::chrono::seconds wait_time_per_attempt;
    static constexpr auto total_attempts = 10;

    ouster::sensor::sensor_info info;
};

}  // namespace ouster_ros
