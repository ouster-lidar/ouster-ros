/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_cloud_nodelet.cpp
 * @brief A nodelet to publish point clouds and imu topics
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
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

#include "ouster_ros/PacketMsg.h"
#include "os_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"
#include "point_cloud_processor_factory.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;

class OusterCloud : public nodelet::Nodelet {
   public:
    OusterCloud() : tf_bcast(getName()) {}

   private:
    virtual void onInit() override {
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

        tf_bcast.parse_parameters(getPrivateNodeHandle());

        auto dynamic_transforms =
            getPrivateNodeHandle().param("dynamic_transforms_broadcast", false);
        auto dynamic_transforms_rate = getPrivateNodeHandle().param(
            "dynamic_transforms_broadcast_rate", 1.0);
        if (dynamic_transforms && dynamic_transforms_rate < 1.0) {
            NODELET_WARN(
                "OusterCloud: dynamic transforms enabled but wrong rate is "
                "set, clamping to 1 Hz!");
            dynamic_transforms_rate = 1.0;
        }

        if (!dynamic_transforms) {
            NODELET_INFO("OusterCloud: using static transforms broadcast");
            tf_bcast.broadcast_transforms(info);
        } else {
            NODELET_INFO_STREAM(
                "OusterCloud: dynamic transforms broadcast enabled wit "
                "broadcast rate of: "
                << dynamic_transforms_rate << " Hz");
            timer_ = getNodeHandle().createTimer(
                ros::Duration(1.0 / dynamic_transforms_rate),
                [this, info](const ros::TimerEvent&) {
                    tf_bcast.broadcast_transforms(info, last_msg_ts);
                });
        }

        create_publishers_subscribers(info);
    }

    void create_publishers_subscribers(const sensor::sensor_info& info) {
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMU|PCL|SCAN"});
        auto tokens = impl::parse_tokens(proc_mask, '|');

        auto timestamp_mode = pnh.param("timestamp_mode", std::string{});
        double ptp_utc_tai_offset = pnh.param("ptp_utc_tai_offset", -37.0);

        auto& nh = getNodeHandle();

        if (impl::check_token(tokens, "IMU")) {
            imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 100);
            imu_packet_handler = ImuPacketHandler::create_handler(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
            imu_packet_sub = nh.subscribe<PacketMsg>(
                "imu_packets", 100, [this](const PacketMsg::ConstPtr msg) {
                    auto imu_msg = imu_packet_handler(msg->buf.data());
                    if (imu_msg.header.stamp > last_msg_ts)
                        last_msg_ts = imu_msg.header.stamp;
                    imu_pub.publish(imu_msg);
                });
        }

        int num_returns = get_n_returns(info);

        std::vector<LidarScanProcessor> processors;
        if (impl::check_token(tokens, "PCL")) {
            lidar_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                lidar_pubs[i] = nh.advertise<sensor_msgs::PointCloud2>(
                    topic_for_return("points", i), 10);
            }

            auto point_type = pnh.param("point_type", std::string{"original"});
            processors.push_back(
                PointCloudProcessorFactory::create_point_cloud_processor(point_type,
                    info, tf_bcast.point_cloud_frame_id(),
                    tf_bcast.apply_lidar_to_sensor_transform(),
                    [this](PointCloudProcessor_OutputType msgs) {
                        for (size_t i = 0; i < msgs.size(); ++i) {
                            if (msgs[i]->header.stamp > last_msg_ts)
                                last_msg_ts = msgs[i]->header.stamp;
                            lidar_pubs[i].publish(*msgs[i]);
                        }
                    }
                )
            );

            // warn about profile incompatibility
            if (PointCloudProcessorFactory::point_type_requires_intensity(point_type) &&
                info.format.udp_profile_lidar == UDPProfileLidar::PROFILE_RNG15_RFL8_NIR8) {
                NODELET_WARN_STREAM(
                    "selected point type '" << point_type << "' is not compatible with the current udp profile: RNG15_RFL8_NIR8");
            }
        }

        if (impl::check_token(tokens, "SCAN")) {
            scan_pubs.resize(num_returns);
            for (int i = 0; i < num_returns; ++i) {
                scan_pubs[i] = nh.advertise<sensor_msgs::LaserScan>(
                    topic_for_return("scan", i), 10);
            }

            // TODO: avoid this duplication in os_cloud_node
            int beams_count = static_cast<int>(get_beams_count(info));
            int scan_ring = pnh.param("scan_ring", 0);
            scan_ring = std::min(std::max(scan_ring, 0), beams_count - 1);
            if (scan_ring != pnh.param("scan_ring", 0)) {
                NODELET_WARN_STREAM(
                    "scan ring is set to a value that exceeds available range "
                    "please choose a value between [0, " << beams_count << "], "
                    "ring value clamped to: " << scan_ring);
            }

            processors.push_back(LaserScanProcessor::create(
                info, tf_bcast.lidar_frame_id(), scan_ring,
                [this](LaserScanProcessor::OutputType msgs) {
                    for (size_t i = 0; i < msgs.size(); ++i) {
                        if (msgs[i]->header.stamp > last_msg_ts)
                            last_msg_ts = msgs[i]->header.stamp;
                        scan_pubs[i].publish(*msgs[i]);
                    }
                }));
        }

        if (impl::check_token(tokens, "PCL") || impl::check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create_handler(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
            lidar_packet_sub = nh.subscribe<PacketMsg>(
                "lidar_packets", 100, [this](const PacketMsg::ConstPtr msg) {
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

    OusterTransformsBroadcaster tf_bcast;

    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;

    ros::Timer timer_;
    ros::Time last_msg_ts;
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterCloud, nodelet::Nodelet)
