/**
 * Copyright (c) 2018-2025, Ouster, Inc.
 * All rights reserved.
 *
 * @file test_msg_analyzers.cpp
 * @brief Tests for message analysis utilities for Ouster sensors
 */


#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>

#include "../src/msg_analyzers.h"

namespace ouster_ros {
namespace test {

class MsgAnalyzersTest : public ::testing::Test {
  protected:
    static void SetUpTestSuite() {
        rclcpp::init(0, nullptr);
    }

    static void TearDownTestSuite() {
        rclcpp::shutdown();
    }

    void SetUp() override {
        node_ = std::make_shared<rclcpp::Node>("test_msg_analyzers");
        clock_ = node_->get_clock();
    }

    void TearDown() override {
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp::Clock::SharedPtr clock_;
};

TEST_F(MsgAnalyzersTest, PointCloudMsgAnalysis) {
    sensor_msgs::msg::PointCloud2 pc_msg;
    pc_msg.header.stamp = clock_->now();
    pc_msg.header.frame_id = "os_sensor";
    pc_msg.height = 64;
    pc_msg.width = 1024;

    auto results = analyze_msg(pc_msg);


    EXPECT_FALSE(results.empty());
}

TEST_F(MsgAnalyzersTest, ImageMsgAnalysis) {
    sensor_msgs::msg::Image img_msg;
    img_msg.header.stamp = clock_->now();
    img_msg.header.frame_id = "os_sensor";

    auto results = analyze_msg(img_msg);
    EXPECT_FALSE(results.empty());
}

TEST_F(MsgAnalyzersTest, RosMsgAggregateTimeAnalyzer) {
    RosMsgAggregateTimeAnalyzer<sensor_msgs::msg::PointCloud2> analyzer(clock_);

    std::vector<sensor_msgs::msg::PointCloud2> msgs;
    auto now = clock_->now();

    for(int i = 0; i < 10; ++i) {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = now + rclcpp::Duration::from_seconds(i * 100);
    }

    std::vector<diagnostic_msgs::msg::KeyValue> last_results;
    for(const auto& msg : msgs) {
        last_results = analyzer(msg);
        EXPECT_FALSE(last_results.empty());
    }
}

TEST_F(MsgAnalyzersTest, AggregateFunction) {
    RingBuffer<detail::MsgTimeInfo> buffer;
    auto now = clock_->now();

    for(int i = 0; i < 5; ++i) {
        detail::MsgTimeInfo info;
        info.msg_timestamp = now + rclcpp::Duration::from_nanoseconds(i * 100000);
        info.received_msg = now + rclcpp::Duration::from_nanoseconds((i * 100 + 10) * 100000);
        buffer.push_back(info);
    }

    auto results = detail::aggregate(buffer);

    EXPECT_FALSE(results.empty());
}

TEST_F(MsgAnalyzersTest, AggregateResultValues) {
    RingBuffer<detail::MsgTimeInfo> buffer;
    auto now = clock_->now();

    // Add entries with known time differences
    for(int i = 0; i < 5; ++i) {
        detail::MsgTimeInfo info;
        info.msg_timestamp = now;
        info.received_msg =
            now + rclcpp::Duration::from_seconds(i + 1); // 1s, 2s, 3s, 4s, 5s differences
        buffer.push_back(info);
    }

    auto results = detail::aggregate(buffer);

    // Find the time_diff_min key-value
    auto min_it = std::find_if(results.begin(), results.end(),
                               [](const auto& kv) { return kv.key == "time_diff_min"; });
    ASSERT_NE(min_it, results.end()) << "time_diff_min not found in results";
    EXPECT_NEAR(std::stod(min_it->value), 1.0, 0.01) << "Min time diff should be 1.0s";

    // Find the time_diff_max key-value
    auto max_it = std::find_if(results.begin(), results.end(),
                               [](const auto& kv) { return kv.key == "time_diff_max"; });
    ASSERT_NE(max_it, results.end()) << "time_diff_max not found in results";
    EXPECT_NEAR(std::stod(max_it->value), 5.0, 0.01) << "Max time diff should be 5.0s";

    // Find the time_diff_avg key-value
    auto avg_it = std::find_if(results.begin(), results.end(),
                               [](const auto& kv) { return kv.key == "time_diff_avg"; });
    ASSERT_NE(avg_it, results.end()) << "time_diff_avg not found in results";
    EXPECT_NEAR(std::stod(avg_it->value), 3.0, 0.01) << "Average time diff should be 3.0s";

    // Check number of frames
    auto frames_it = std::find_if(results.begin(), results.end(),
                                  [](const auto& kv) { return kv.key == "time_diff_frames"; });
    ASSERT_NE(frames_it, results.end()) << "time_diff_frames not found in results";
    EXPECT_EQ(std::stoi(frames_it->value), 5) << "Frame count should be 5";
}


} // namespace test
} // namespace ouster_ros