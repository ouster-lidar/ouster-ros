/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_image_nodelet.cpp
 * @brief A nodelet to decode range, near ir and signal images from ouster
 * point cloud
 *
 * Publishes ~/range_image, ~/nearir_image, and ~/signal_image.  Please bear
 * in mind that there is rounding/clamping to display 8 bit images. For computer
 * vision applications, use higher bit depth values in /os_cloud_node/points
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/String.h>

#include "lidar_packet_handler.h"
#include "image_processor.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;


class OusterImage : public nodelet::Nodelet {
   private:
    virtual void onInit() override {
        create_metadata_subscriber(getNodeHandle());
        NODELET_INFO("OusterImage: node initialized!");
    }

    void create_metadata_subscriber(ros::NodeHandle& nh) {
        metadata_sub = nh.subscribe<std_msgs::String>(
            "metadata", 1, &OusterImage::metadata_handler, this);
    }

   private:
    void metadata_handler(const std_msgs::String::ConstPtr& metadata_msg) {
        NODELET_INFO("OusterImage: retrieved new sensor metadata!");
        info = sensor::parse_metadata(metadata_msg->data);
        create_publishers_subscribers(get_n_returns(info));
    }

    void create_publishers_subscribers(int n_returns) {
        // TODO: avoid having to replicate the parameters:
        // timestamp_mode, ptp_utc_tai_offset, use_system_default_qos in yet
        // another node.
        auto& pnh = getPrivateNodeHandle();
        auto timestamp_mode = pnh.param("timestamp_mode", std::string{});
        double ptp_utc_tai_offset = pnh.param("ptp_utc_tai_offset", -37.0);

        const std::map<sensor::ChanField, std::string>
            channel_field_topic_map_1 {
                {sensor::ChanField::RANGE, "range_image"},
                {sensor::ChanField::SIGNAL, "signal_image"},
                {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                {sensor::ChanField::NEAR_IR, "nearir_image"}};

        const std::map<sensor::ChanField, std::string>
            channel_field_topic_map_2 {
                {sensor::ChanField::RANGE, "range_image"},
                {sensor::ChanField::SIGNAL, "signal_image"},
                {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                {sensor::ChanField::NEAR_IR, "nearir_image"},
                {sensor::ChanField::RANGE2, "range_image2"},
                {sensor::ChanField::SIGNAL2, "signal_image2"},
                {sensor::ChanField::REFLECTIVITY2, "reflec_image2"}};

        auto which_map = n_returns == 1 ? &channel_field_topic_map_1
                                        : &channel_field_topic_map_2;

        auto& nh = getNodeHandle();

        for (auto it = which_map->begin(); it != which_map->end(); ++it) {
            image_pubs[it->first] =
                nh.advertise<sensor_msgs::Image>(it->second, 100);
        }

        std::vector<LidarScanProcessor> processors {
            ImageProcessor::create(
                info, "os_lidar", /*TODO: tf_bcast.point_cloud_frame_id()*/
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first].publish(*it->second);
                    }
                })
        };

        const int min_lidar_packets_per_cloud =
            pnh.param("min_lidar_packets_per_cloud", 0);
        lidar_packet_handler = LidarPacketHandler::create_handler(
            info, processors, timestamp_mode,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            min_lidar_packets_per_cloud);
        lidar_packet_sub = nh.subscribe<PacketMsg>(
                "lidar_packets", 100,
                [this](const PacketMsg::ConstPtr msg) {
                    lidar_packet_handler(msg->buf.data());
                });
    }

   private:
    ros::Subscriber metadata_sub;
    sensor::sensor_info info;

    ros::Subscriber lidar_packet_sub;
    std::map<sensor::ChanField, ros::Publisher> image_pubs;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};
}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterImage, nodelet::Nodelet)
