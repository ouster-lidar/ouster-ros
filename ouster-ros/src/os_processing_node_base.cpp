/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.cpp
 * @brief implementation of OusterProcessingNodeBase interface
 */

#include "ouster_ros/os_processing_node_base.h"

using ouster::sensor::UDPProfileLidar;
using ouster_srvs::srv::GetMetadata;
using namespace std::chrono_literals;

namespace ouster_ros {

bool OusterProcessingNodeBase::wait_for_get_metadata_service(
    std::shared_ptr<rclcpp::Client<GetMetadata>> client) {
    // TODO: Add as node parameteres?
    constexpr auto wait_time_per_attempt = 5s;
    constexpr auto total_attempts = 5;
    int remaining_attempts = total_attempts;
    bool service_available;
    do {
        RCLCPP_INFO_STREAM(get_logger(),
                           "contacting get_metadata service, attempt no: "
                               << total_attempts - remaining_attempts + 1 << "/"
                               << total_attempts);
        service_available = client->wait_for_service(wait_time_per_attempt);
    } while (!service_available && --remaining_attempts > 0);
    return service_available;
}

std::string OusterProcessingNodeBase::get_metadata() {
    auto client = create_client<GetMetadata>("get_metadata");
    if (!wait_for_get_metadata_service(client)) {
        auto error_msg = "get_metadata service is unavailable";
        RCLCPP_ERROR_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }
    auto request = std::make_shared<GetMetadata::Request>();
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result,
                                           10s) !=
        rclcpp::FutureReturnCode::SUCCESS) {
        auto error_msg = "Calling get_metadata service failed";
        RCLCPP_ERROR_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }

    RCLCPP_INFO(get_logger(), "retrieved sensor metadata!");
    return result.get()->metadata;
}

int OusterProcessingNodeBase::get_n_returns() {
    return info.format.udp_profile_lidar ==
                   UDPProfileLidar::PROFILE_RNG19_RFL8_SIG16_NIR16_DUAL
               ? 2
               : 1;
}

}  // namespace ouster_ros
