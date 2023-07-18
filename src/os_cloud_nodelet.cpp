/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_nodelet.cpp
 * @brief A nodelet to publish point clouds and imu topics
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <cassert>

#include "ouster_ros/PacketMsg.h"
#include "os_transforms_broadcaster.h"

#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;

namespace nodelets_os {

class OusterCloud : public nodelet::Nodelet {
   public:
    OusterCloud() : tf_bcast(getName()) {
    }

   private:
    static bool is_arg_set(const std::string& arg) {
        return arg.find_first_not_of(' ') != std::string::npos;
    }

    virtual void onInit() override {
        tf_bcast.parse_parameters(getPrivateNodeHandle());
        create_metadata_subscriber();
        NODELET_INFO("OusterCloud: nodelet created!");
    }

    void create_metadata_subscriber() {
        metadata_sub = getNodeHandle().subscribe<std_msgs::String>(
            "metadata", 1, &OusterCloud::metadata_handler, this);
    }

    void metadata_handler(const std_msgs::String::ConstPtr& metadata_msg) {
        // TODO: handle sensor reconfigurtion
        NODELET_INFO("OusterCloud: retrieved new sensor metadata!");
        auto info = sensor::parse_metadata(metadata_msg->data);
        // NOTE using the static broadcast form for now!
        tf_bcast.broadcast_transforms(info);
        create_publishers(get_n_returns(info));
        create_subscriptions(info);
    }

    void create_publishers(int num_returns) {
        auto& nh = getNodeHandle();
        imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
        lidar_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; ++i) {
            lidar_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>(
                ouster_ros::topic_for_return("points", i), 10);
        }

        scan_pubs.resize(num_returns);
        for (int i = 0; i < num_returns; i++) {
            scan_pubs[i] = nh.advertise<sensor_msgs::LaserScan>(
                ouster_ros::topic_for_return("scan", i), 10);
        }
    }

    void create_subscriptions(const sensor::sensor_info& info) {

        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMU|PCL|SCAN"});
        auto tokens = parse_tokens(proc_mask, '|');

        auto timestamp_mode_arg = pnh.param("timestamp_mode", std::string{});
        bool use_ros_time = timestamp_mode_arg == "TIME_FROM_ROS_TIME";

        auto& nh = getNodeHandle();

        if (check_token(tokens, "IMU")) {
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, tf_bcast.imu_frame_id(), use_ros_time);
            imu_packet_sub = nh.subscribe<PacketMsg>(
                "imu_packets", 100,
                [this](const PacketMsg::ConstPtr msg) {
                    auto imu_msg = imu_packet_handler(msg->buf.data());
                    imu_pub.publish(imu_msg);
                });
        }

        std::vector<LidarScanProcessor> processors;
        if (check_token(tokens, "PCL")) {
            processors.push_back(PointCloudProcessor::create(
                info, tf_bcast.point_cloud_frame_id(),
                tf_bcast.apply_lidar_to_sensor_transform(),
                [this](PointCloudProcessor::OutputType point_cloud_msg) {
                    for (size_t i = 0; i < point_cloud_msg.size(); ++i)
                        lidar_pubs[i].publish(*point_cloud_msg[i]);
                }));
        }

        if (check_token(tokens, "SCAN")) {
            // TODO: avoid duplication in os_cloud_node
            int beams_count = static_cast<int>(ouster_ros::get_beams_count(info));
            int scan_ring = pnh.param("scan_ring", 0);
            scan_ring = std::min(std::max(scan_ring, 0), beams_count - 1);
            if (scan_ring != pnh.param("scan_ring", 0)) {
                ROS_WARN_STREAM(
                    "scan ring is set to a value that exceeds available range"
                    "please choose a value between [0, " << beams_count << "], "
                    "ring value clamped to: " << scan_ring);
            }

            processors.push_back(LaserScanProcessor::create(
                info,
                tf_bcast
                    .point_cloud_frame_id(),  // TODO: should we allow having a
                                              // different frame for the laser
                                              // scan than point cloud???
                0, [this](LaserScanProcessor::OutputType laser_scan_msg) {
                    for (size_t i = 0; i < laser_scan_msg.size(); ++i)
                        scan_pubs[i].publish(*laser_scan_msg[i]);
                }));
        }

        if (check_token(tokens, "PCL") || check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, use_ros_time, processors);
            lidar_packet_sub = nh.subscribe<PacketMsg>(
                "lidar_packets", 100,
                [this](const PacketMsg::ConstPtr msg) {
                    lidar_packet_handler(msg->buf.data());
                });
        }
    }

   private:
    ros::Subscriber metadata_sub;
    ros::Subscriber imu_packet_sub;
    ros::Publisher imu_pub;
    ros::Subscriber lidar_packet_sub;
    std::vector<ros::Publisher> lidar_pubs;
    std::vector<ros::Publisher> scan_pubs;

    ouster_ros::OusterTransformsBroadcaster tf_bcast;

    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros


PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterCloud, nodelet::Nodelet)
