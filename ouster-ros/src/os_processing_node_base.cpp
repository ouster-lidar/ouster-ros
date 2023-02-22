/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_processing_node_base.cpp
 * @brief implementation of OusterProcessingNodeBase interface
 */

#include "ouster_ros/os_processing_node_base.h"

using namespace std::chrono_literals;
using ouster::sensor::UDPProfileLidar;
using ouster_srvs::srv::GetMetadata;
using rclcpp::FutureReturnCode;

namespace ouster_ros {

bool OusterProcessingNodeBase::spin_till_attempts_exahused(
    const std::string& log_msg, std::function<bool(void)> lambda) {
    int remaining_attempts = total_attempts;
    bool done;
    do {
        RCLCPP_INFO_STREAM(get_logger(),
                           log_msg << "; attempt no: "
                                   << total_attempts - remaining_attempts + 1
                                   << "/" << total_attempts);
        done = lambda();
    } while (rclcpp::ok() && !done && --remaining_attempts > 0);
    return done;
}

std::string OusterProcessingNodeBase::get_metadata() {
    auto client = create_client<GetMetadata>("get_metadata");
    if (!spin_till_attempts_exahused(
            "contacting get_metadata service", [this, &client]() -> bool {
                return client->wait_for_service(wait_time_per_attempt);
            })) {
        auto error_msg = "get_metadata service is unavailable";
        RCLCPP_ERROR_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }
    auto request = std::make_shared<GetMetadata::Request>();
    auto result = client->async_send_request(request);

    rclcpp::FutureReturnCode return_code;
    spin_till_attempts_exahused(
        "waiting for get_metadata service to respond",
        [this, &return_code, &result]() -> bool {
            return_code = rclcpp::spin_until_future_complete(
                get_node_base_interface(), result, wait_time_per_attempt);
            return return_code != FutureReturnCode::TIMEOUT;
        });

    if (return_code != FutureReturnCode::SUCCESS) {
        auto error_msg = "get_metadata service timed out or interrupted";
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

const std::chrono::seconds OusterProcessingNodeBase::wait_time_per_attempt = std::chrono::seconds(10);
    
}  // namespace ouster_ros
