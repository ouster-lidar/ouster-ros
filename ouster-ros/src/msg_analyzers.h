/**
 * Copyright (c) 2018-2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file msg_analyzers.h
 * @brief Message analysis utilities for Ouster sensors
 */

#pragma once

#include <algorithm>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <ouster_ros/ring_buffer.h>
#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/node.hpp>

#include <vector>

namespace ouster_ros
{

std::vector<diagnostic_msgs::msg::KeyValue> analyze_msg(const sensor_msgs::msg::PointCloud2 & msg);
std::vector<diagnostic_msgs::msg::KeyValue> analyze_msg(const sensor_msgs::msg::Image & msg);

namespace detail
{
struct MsgTimeInfo
{
  rclcpp::Time msg_timestamp;
  rclcpp::Time received_msg;
};

std::vector<diagnostic_msgs::msg::KeyValue> aggregate(
  const RingBuffer<MsgTimeInfo> time_info_buffer);
}  //namespace detail

template <typename MsgT>
class RosMsgAggregateTimeAnalyzer
{
public:
  RosMsgAggregateTimeAnalyzer() = default;
  RosMsgAggregateTimeAnalyzer(rclcpp::Clock::SharedPtr clock) : clock_(clock) {}

  std::vector<diagnostic_msgs::msg::KeyValue> operator()(const MsgT & msg)
  {
    detail::MsgTimeInfo time_info{msg.header.stamp, clock_->now()};
    time_info_buffer_.push_back(time_info);
    return detail::aggregate(time_info_buffer_);
  }

private:
  rclcpp::Clock::SharedPtr clock_;

  RingBuffer<detail::MsgTimeInfo> time_info_buffer_;
};

}  // namespace ouster_ros
