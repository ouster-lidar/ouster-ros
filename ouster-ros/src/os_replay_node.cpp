/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_replay_node.cpp
 * @brief This node mainly handles publishing saved metadata
 *
 */

#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_client_base_node.h"


namespace sensor = ouster::sensor;

namespace ouster_ros {

class OusterReplay : public OusterClientBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterReplay(const rclcpp::NodeOptions& options)
    : OusterClientBase("os_replay", options) {
        onInit();
    }

   private:
    virtual void onInit() override {
        declare_parameter("metadata", rclcpp::PARAMETER_STRING);
        auto meta_file = get_parameter("metadata").as_string();
        if (!is_arg_set(meta_file)) {
            RCLCPP_ERROR(get_logger(), "Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }

        RCLCPP_INFO(get_logger(), "Running in replay mode");

        // populate info for config service
        try {
            info = sensor::metadata_from_json(meta_file);
            cached_metadata = to_string(info);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(get_logger(),
            "Error when running in replay mode: %s", e.what());
        }

        OusterClientBase::onInit();
    }
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterReplay)
