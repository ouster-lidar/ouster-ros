/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_replay_node.cpp
 * @brief This node mainly handles publishing saved metadata
 *
 */

#include "ouster_ros/os_sensor_node_base.h"
#include "ouster_ros/visibility_control.h"

namespace sensor = ouster::sensor;

namespace ouster_ros {

class OusterReplay : public OusterSensorNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterReplay(const rclcpp::NodeOptions& options)
        : OusterSensorNodeBase("os_replay", options) {
        on_init();
    }

   private:
    void on_init() {
        declare_parameters();
        auto meta_file = parse_parameters();
        populate_metadata_from_json(meta_file);
        create_get_metadata_service();
        RCLCPP_INFO(get_logger(), "Running in replay mode");
    }

    void declare_parameters() {
        declare_parameter("metadata", rclcpp::PARAMETER_STRING);
    }

    std::string parse_parameters() {
        auto meta_file = get_parameter("metadata").as_string();
        if (!is_arg_set(meta_file)) {
            RCLCPP_ERROR(get_logger(),
                         "Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }
        return meta_file;
    }

    void populate_metadata_from_json(const std::string& meta_file) {
        try {
            info = sensor::metadata_from_json(meta_file);
            cached_metadata = to_string(info);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Error when running in replay mode: " << e.what());
        }
    }
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterReplay)
