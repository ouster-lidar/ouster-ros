/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node_base.cpp
 * @brief implementation of OusterSensorNodeBase interface
 */

#include "ouster_ros/os_sensor_node_base.h"
#include <ouster/impl/build.h>
#include <fstream>
#include <lifecycle_msgs/msg/transition.hpp>

using namespace std::string_literals;
using lifecycle_msgs::srv::ChangeState;
namespace sensor = ouster::sensor;
using ouster_sensor_msgs::srv::GetMetadata;
using sensor::UDPProfileLidar;

namespace ouster_ros {

OusterSensorNodeBase::OusterSensorNodeBase(
    const std::string& name, const rclcpp::NodeOptions& options)
: rclcpp_lifecycle::LifecycleNode(name, options),
  change_state_client{create_client<ChangeState>(name + "/change_state"s)} {
}


void OusterSensorNodeBase::create_get_metadata_service() {
    get_metadata_srv = create_service<GetMetadata>(
        "get_metadata",
        [this](const std::shared_ptr<GetMetadata::Request>,
               std::shared_ptr<GetMetadata::Response> response) {
            response->metadata = cached_metadata;
            return cached_metadata.size() > 0;
        });

    RCLCPP_INFO(get_logger(), "get_metadata service created");
}

void OusterSensorNodeBase::create_metadata_pub() {
    auto latching_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    latching_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    latching_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    metadata_pub =
        create_publisher<std_msgs::msg::String>("metadata", latching_qos);
}

void OusterSensorNodeBase::publish_metadata() {
    std_msgs::msg::String metadata_msg;
    metadata_msg.data = cached_metadata;
    metadata_pub->publish(metadata_msg);
}

void OusterSensorNodeBase::display_lidar_info(const sensor::sensor_info& info) {
    auto lidar_profile = info.format.udp_profile_lidar;
    RCLCPP_INFO_STREAM(
        get_logger(),
        "ouster client version: "
            << ouster::SDK_VERSION_FULL << "\n"
            << "product: " << info.prod_line << ", sn: " << info.sn << ", "
            << "firmware rev: " << info.fw_rev << "\n"
            << "lidar mode: " << sensor::to_string(info.mode) << ", "
            << "lidar udp profile: " << sensor::to_string(lidar_profile));
}

std::string OusterSensorNodeBase::read_text_file(const std::string& text_file) {
    std::ifstream ifs{};
    ifs.open(text_file);
    if (ifs.fail()) return {};
    std::stringstream buf;
    buf << ifs.rdbuf();
    return buf.str();
}

bool OusterSensorNodeBase::write_text_to_file(const std::string& file_path,
                                              const std::string& text) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) return false;
    ofs << text << std::endl;
    ofs.close();
    return true;
}

std::string OusterSensorNodeBase::transition_id_to_string(uint8_t transition_id) {
    switch (transition_id) {
        case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
            return "create"s;
        case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
            return "configure"s;
        case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
            return "cleanup"s;
        case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
            return "activate"s;
        case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
            return "deactivate"s;
        case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
            return "destroy"s;
        default:
            return "unknown"s;
    }
}

template <typename CallbackT, typename... CallbackT_Args>
bool OusterSensorNodeBase::change_state(std::uint8_t transition_id, CallbackT callback,
                                CallbackT_Args... callback_args,
                                std::chrono::seconds time_out) {
    if (!change_state_client->wait_for_service(time_out)) {
        RCLCPP_ERROR_STREAM(
            get_logger(), "Service " << change_state_client->get_service_name()
                                    << "is not available.");
        return false;
    }

    auto request = std::make_shared<ChangeState::Request>();
    request->transition.id = transition_id;
    // send an async request to perform the transition
    change_state_client->async_send_request(
        request, [this, callback,
                callback_args...](rclcpp::Client<ChangeState>::SharedFuture ff) {
            callback(callback_args..., ff.get()->success);
        });
    return true;
}

void OusterSensorNodeBase::execute_transitions_sequence(
    std::vector<uint8_t> transitions_sequence, size_t at) {
    assert(at < transitions_sequence.size() &&
        "at index exceeds the number of transitions");
    auto transition_id = transitions_sequence[at];
    RCLCPP_DEBUG_STREAM(
        get_logger(), "transition: [" << transition_id_to_string(transition_id)
                                    << "] started");
    change_state(transition_id, [this, transitions_sequence, at](bool success) {
        if (success) {
            RCLCPP_DEBUG_STREAM(
                get_logger(),
                "transition: [" << transition_id_to_string(transitions_sequence[at])
                                << "] completed");
            if (at < transitions_sequence.size() - 1) {
                execute_transitions_sequence(transitions_sequence, at + 1);
            } else {
                RCLCPP_DEBUG_STREAM(get_logger(), "transitions sequence completed");
            }
        } else {
            RCLCPP_DEBUG_STREAM(
                get_logger(),
                "transition: [" << transition_id_to_string(transitions_sequence[at])
                                << "] failed");
        }
    });
}

}  // namespace ouster_ros
