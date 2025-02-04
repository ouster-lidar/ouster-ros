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
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMG"});
        auto tokens = impl::parse_tokens(proc_mask, '|');
        if (impl::check_token(tokens, "IMG")) {
            create_lidar_packets_subscriber();
            create_image_publishers();
        }
        create_metadata_subscriber();
        NODELET_INFO("OusterImage: node initialized!");
    }

    void create_metadata_subscriber() {
        metadata_sub = getNodeHandle().subscribe<std_msgs::String>(
            "metadata", 1, &OusterImage::metadata_handler, this);
    }

    void create_lidar_packets_subscriber() {
        lidar_packet_sub = getNodeHandle().subscribe<PacketMsg>(
            "lidar_packets", 100, [this](const PacketMsg::ConstPtr msg) {
                if (lidar_packet_handler) {
                    // TODO[UN]: this is not ideal since we can't reuse the msg buffer
                    // Need to redefine the Packet object and allow use of array_views
                    sensor::LidarPacket lidar_packet(msg->buf.size());
                    memcpy(lidar_packet.buf.data(), msg->buf.data(), msg->buf.size());
                    lidar_packet.host_timestamp = static_cast<uint64_t>(ros::Time::now().toNSec());
                    lidar_packet_handler(lidar_packet);
                }
        });
    }

    void create_image_publishers() {
        // NOTE: always create the 2nd topics
        const std::map<sensor::ChanField, std::string>
            channel_field_topic_map {
                {sensor::ChanField::RANGE, "range_image"},
                {sensor::ChanField::SIGNAL, "signal_image"},
                {sensor::ChanField::REFLECTIVITY, "reflec_image"},
                {sensor::ChanField::NEAR_IR, "nearir_image"},
                {sensor::ChanField::RANGE2, "range_image2"},
                {sensor::ChanField::SIGNAL2, "signal_image2"},
                {sensor::ChanField::REFLECTIVITY2, "reflec_image2"}};

        for (auto it : channel_field_topic_map) {
            image_pubs[it.first] = getNodeHandle().advertise<sensor_msgs::Image>(it.second, 100);
        }
    }

    void metadata_handler(const std_msgs::String::ConstPtr& metadata_msg) {
        NODELET_INFO("OusterImage: retrieved new sensor metadata!");
        auto info = sensor::parse_metadata(metadata_msg->data);

        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMG"});
        auto tokens = impl::parse_tokens(proc_mask, '|');
        if (impl::check_token(tokens, "IMG"))
            create_handlers(info);
    }

    void create_handlers(const sensor::sensor_info& info) {
        // TODO: avoid having to replicate the parameters: 
        // timestamp_mode, ptp_utc_tai_offset, use_system_default_qos in yet
        // another node.
        auto& pnh = getPrivateNodeHandle();
        auto timestamp_mode = pnh.param("timestamp_mode", std::string{});
        double ptp_utc_tai_offset = pnh.param("ptp_utc_tai_offset", -37.0);

        auto min_scan_valid_columns_ratio = pnh.param("min_scan_valid_columns_ratio", 0.0f);
        if (min_scan_valid_columns_ratio < 0.0f || min_scan_valid_columns_ratio > 1.0f) {
            NODELET_FATAL("min_scan_valid_columns_ratio needs to be in the range [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio out of bounds!");
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

        lidar_packet_handler = LidarPacketHandler::create(
            info, processors, timestamp_mode,
            static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
            min_scan_valid_columns_ratio);
    }

   private:
    ros::Subscriber metadata_sub;
    ros::Subscriber lidar_packet_sub;
    std::map<sensor::ChanField, ros::Publisher> image_pubs;

    LidarPacketHandler::HandlerType lidar_packet_handler;
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterImage, nodelet::Nodelet)
