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
        declare_parameters();
    }

    LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) {
        RCLCPP_INFO(get_logger(), "on_configure() is called.");

        try {
            auto meta_file = parse_parameters();
            create_metadata_publisher();
            load_metadata_from_file(meta_file);
            publish_metadata();
            create_get_metadata_service();
            RCLCPP_INFO(get_logger(), "Running in replay mode");
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "exception thrown while configuring the sensor, details: "
                    << ex.what());
            // TODO: return ERROR on fatal errors, FAILURE otherwise
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_activate() is called.");
        LifecycleNode::on_activate(state);
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_error(
        const rclcpp_lifecycle::State&) {
        RCLCPP_DEBUG(get_logger(), "on_error() is called.");
        // Always return failure for now
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");
        LifecycleNode::on_deactivate(state);
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) {
        RCLCPP_DEBUG(get_logger(), "on_cleanup() is called.");
        cleanup();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");

        if (state.label() == "unconfigured") {
            // nothing to do, return success
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        // whether state was 'active' or 'inactive' do cleanup
        try {
            cleanup();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "exception thrown durng cleanup, details: " << ex.what());
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

   private:
    void declare_parameters() { declare_parameter<std::string>("metadata"); }

    std::string parse_parameters() {
        auto meta_file = get_parameter("metadata").as_string();
        if (!is_arg_set(meta_file)) {
            RCLCPP_ERROR(get_logger(),
                         "Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }
        return meta_file;
    }

    void load_metadata_from_file(const std::string& meta_file) {
        try {
            cached_metadata = read_text_file(meta_file);
            info = sensor::parse_metadata(cached_metadata);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            cached_metadata.clear();
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Error when running in replay mode: " << e.what());
        }
    }

    void cleanup() { get_metadata_srv.reset(); }
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterReplay)
