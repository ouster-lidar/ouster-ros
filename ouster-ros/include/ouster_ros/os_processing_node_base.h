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
#include <cstdint>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace ouster_ros {

class OusterProcessingNodeBase : public rclcpp::Node {
   protected:
    OusterProcessingNodeBase(const std::string& name,
                             const rclcpp::NodeOptions& options)
        : rclcpp::Node(name, options) {}

    void create_metadata_subscriber(
        std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>
            on_sensor_metadata);

    // Reads the optional "metadata" file-path parameter; when it is set, loads
    // the file and invokes on_sensor_metadata with its contents. This lets the
    // processing nodes obtain sensor metadata without the latched /metadata
    // topic (e.g. a plain `ros2 bag play` of recorded packets, where the
    // transient-local topic is not reliably re-delivered). Returns true if
    // metadata was loaded from a file.
    bool load_metadata_from_file(
        const std::function<void(const std_msgs::msg::String::ConstSharedPtr&)>&
            on_sensor_metadata);

    // Metadata and packet subscriptions currently share the node's default
    // mutually-exclusive callback group. Keep an explicit lock as well so a
    // future callback-group change cannot race pipeline teardown against an
    // in-flight packet callback.
    std::mutex pipeline_mutex;

    bool metadata_is_active(const std::string& metadata) const {
        return has_active_metadata_ && metadata == active_metadata_;
    }

    void mark_metadata_active(const std::string& metadata) {
        active_metadata_ = metadata;
        has_active_metadata_ = true;
    }

    void invalidate_active_metadata() {
        active_metadata_.clear();
        has_active_metadata_ = false;
    }

    uint64_t begin_pipeline_update() {
        invalidate_active_metadata();
        return ++pipeline_generation_;
    }

    bool pipeline_is_current(uint64_t generation) const {
        return generation == pipeline_generation_;
    }

   protected:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr metadata_sub;
    ouster::sdk::core::SensorInfo info;
    std::shared_ptr<ouster::sdk::core::PacketFormat> packet_format;

   private:
    std::string active_metadata_;
    bool has_active_metadata_ = false;
    uint64_t pipeline_generation_ = 0;
};

}  // namespace ouster_ros
