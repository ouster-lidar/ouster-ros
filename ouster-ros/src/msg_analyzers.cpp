/**
 * Copyright (c) 2018-2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file msg_analyzers.cpp
 * @brief Implementation of message analysis utilities for Ouster sensors
 */
#include "msg_analyzers.h"
#include <chrono>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <string>

namespace ouster_ros {


inline diagnostic_msgs::msg::KeyValue to_kv(const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    return kv;
}

template <typename MsgWithHeaderT>
auto to_sec(const MsgWithHeaderT& time) {
    return std::chrono::seconds(time.sec) + std::chrono::nanoseconds(time.nanosec);
}

inline std::string to_string(const builtin_interfaces::msg::Time& header) {
    auto time = to_sec(header);
    return std::to_string(time.count());
}

template <typename MsgT>
std::vector<diagnostic_msgs::msg::KeyValue> add_timestamp(const MsgT& msg) {
    std::vector<diagnostic_msgs::msg::KeyValue> key_values;
    key_values.emplace_back(to_kv("timestamp", to_string(msg.header.stamp)));
    return key_values;
}


std::vector<diagnostic_msgs::msg::KeyValue> analyze_msg(const sensor_msgs::msg::PointCloud2& msg) {
    return add_timestamp(msg);
}

std::vector<diagnostic_msgs::msg::KeyValue> analyze_msg(const sensor_msgs::msg::Image& msg) {
    return add_timestamp(msg);
}

template <typename MsgT>
std::vector<diagnostic_msgs::msg::KeyValue> add_time_diff(const rclcpp::Clock::SharedPtr clock,
                                                          const MsgT& msg) {
    auto result = add_timestamp(msg);
    if(clock) {
        const auto current_time = clock->now();
        const auto msg_time = msg.header.stamp;
        const auto time_diff = current_time - msg_time;
        result.emplace_back(to_kv("current_time", std::to_string(current_time.seconds())));
        result.emplace_back(to_kv("time_diff", std::to_string(time_diff.seconds())));
    }
    return result;
}

auto time_diff(const detail::MsgTimeInfo& info) {
    return (info.received_msg - info.msg_timestamp);
}


namespace detail {
std::vector<diagnostic_msgs::msg::KeyValue>
aggregate(const RingBuffer<MsgTimeInfo> time_info_buffer_) {

    std::vector<diagnostic_msgs::msg::KeyValue> key_values;

    const auto [time_diff_min, time_diff_max] = std::minmax_element(
        time_info_buffer_.begin(), time_info_buffer_.end(),
        [](const auto& a, const auto& b) { return time_diff(a) < time_diff(b); });
    if(time_diff_min != time_info_buffer_.end()) {
        key_values.emplace_back(
            to_kv("time_diff_min", std::to_string(time_diff(*time_diff_min).seconds())));
    }

    if(time_diff_max != time_info_buffer_.end()) {
        key_values.emplace_back(
            to_kv("time_diff_max", std::to_string(time_diff(*time_diff_max).seconds())));
    }
    const auto time_diff_sum =
        std::accumulate(time_info_buffer_.begin(), time_info_buffer_.end(), rclcpp::Duration{0, 0},
                        [](const auto& sum, const auto& info) { return sum + time_diff(info); });

    const auto time_diff_avg = time_diff_sum.seconds() / time_info_buffer_.size();

    key_values.emplace_back(to_kv("time_diff_avg", std::to_string(time_diff_avg)));
    key_values.emplace_back(to_kv("time_diff_frames", std::to_string(time_info_buffer_.size())));
    return key_values;
}

} // namespace detail

} // namespace ouster_ros
