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
#include "telemetry_handler.h"

namespace ouster_ros {

namespace sensor = ouster::sensor;
using ouster_ros::PacketMsg;

class OusterCloud : public nodelet::Nodelet {
   public:
    OusterCloud() : tf_bcast(getName()) {}

   private:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMU|PCL|SCAN"});
        auto tokens = impl::parse_tokens(proc_mask, '|');
        if (impl::check_token(tokens, "IMU")) create_imu_pub_sub();
        if (impl::check_token(tokens, "PCL")) create_point_cloud_pubs();
        if (impl::check_token(tokens, "SCAN")) create_laser_scan_pubs();
        if (impl::check_token(tokens, "TLM")) create_telemetry_pub();
        if (impl::check_token(tokens, "PCL") ||
            impl::check_token(tokens, "SCAN") ||
            impl::check_token(tokens, "TLM"))
            create_lidar_packets_sub();
        create_metadata_subscriber();
        NODELET_INFO("OusterCloud: nodelet created!");
    }

    void create_metadata_subscriber() {
        metadata_sub = getNodeHandle().subscribe<std_msgs::String>(
            "metadata", 1, &OusterCloud::metadata_handler, this);
    }

    void metadata_handler(const std_msgs::String::ConstPtr& metadata_msg) {
        NODELET_INFO("OusterCloud: retrieved new sensor metadata!");
        auto info = sensor::parse_metadata(metadata_msg->data);

        auto pnh = getPrivateNodeHandle();
        tf_bcast.parse_parameters(pnh);
        auto dynamic_transforms =
            pnh.param("dynamic_transforms_broadcast", false);
        auto dynamic_transforms_rate = getPrivateNodeHandle().param(
            "dynamic_transforms_broadcast_rate", 1.0);
        if (dynamic_transforms && dynamic_transforms_rate < 1.0) {
            NODELET_WARN(
                "OusterCloud: dynamic transforms enabled but wrong rate is "
                "set, clamping to 1 Hz!");
            dynamic_transforms_rate = 1.0;
        }

        if (tf_bcast.publish_static_tf()) {
            if (!dynamic_transforms) {
                NODELET_INFO("OusterCloud: using static transforms broadcast");
                tf_bcast.broadcast_transforms(info);
            } else {
                NODELET_INFO_STREAM(
                    "OusterCloud: dynamic transforms broadcast enabled with "
                    "broadcast rate of: "
                    << dynamic_transforms_rate << " Hz");
                timer_.stop();
                timer_ = getNodeHandle().createTimer(
                    ros::Duration(1.0 / dynamic_transforms_rate),
                    [this, info](const ros::TimerEvent&) {
                        tf_bcast.broadcast_transforms(info, last_msg_ts);
                    });
            }
        }

        create_handlers(info);
    }

    void create_imu_pub_sub() {
        imu_pub = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 100);
        imu_packet_sub = getNodeHandle().subscribe<PacketMsg>(
            "imu_packets", 100, [this](const PacketMsg::ConstPtr msg) {
                if (imu_packet_handler) {
                    // TODO[UN]: this is not ideal since we can't reuse the msg
                    // buffer Need to redefine the Packet object and allow use
                    // of array_views
                    sensor::ImuPacket imu_packet(msg->buf.size());
                    memcpy(imu_packet.buf.data(), msg->buf.data(),
                           msg->buf.size());
                    imu_packet.host_timestamp =
                        static_cast<uint64_t>(ros::Time::now().toNSec());
                    auto imu_msg = imu_packet_handler(imu_packet);
                    if (imu_msg.header.stamp > last_msg_ts)
                        last_msg_ts = imu_msg.header.stamp;
                    imu_pub.publish(imu_msg);
                }
            });
    }

    void create_point_cloud_pubs() {
        // NOTE: always create the 2nd topic
        lidar_pubs.resize(2);
        for (int i = 0; i < 2; ++i) {
            lidar_pubs[i] = getNodeHandle().advertise<sensor_msgs::PointCloud2>(
                topic_for_return("points", i), 10);
        }
    }

    void create_laser_scan_pubs() {
        // NOTE: always create the 2nd topic
        scan_pubs.resize(2);
        for (int i = 0; i < 2; ++i) {
            scan_pubs[i] = getNodeHandle().advertise<sensor_msgs::LaserScan>(
                topic_for_return("scan", i), 10);
        }
    }

    void create_telemetry_pub() {
        telemetry_pub =
            getNodeHandle().advertise<ouster_ros::Telemetry>("telemetry", 1280);
    }

    void create_lidar_packets_sub() {
        lidar_packet_sub = getNodeHandle().subscribe<PacketMsg>(
            "lidar_packets", 100, [this](const PacketMsg::ConstPtr msg) {
                sensor::LidarPacket lidar_packet(msg->buf.size());
                memcpy(lidar_packet.buf.data(), msg->buf.data(),
                       msg->buf.size());
                lidar_packet.host_timestamp =
                    static_cast<uint64_t>(ros::Time::now().toNSec());

                if (telemetry_handler) {
                    auto telemetry = telemetry_handler(lidar_packet);
                    telemetry_pub.publish(telemetry);
                }

                if (lidar_packet_handler) {
                    // TODO[UN]: this is not ideal since we can't reuse the msg
                    // buffer Need to redefine the Packet object and allow use
                    // of array_views
                    lidar_packet_handler(lidar_packet);
                }
            });
    }

    void create_handlers(const sensor::sensor_info& info) {
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask = pnh.param("proc_mask", std::string{"IMU|PCL|SCAN"});
        auto tokens = impl::parse_tokens(proc_mask, '|');

        auto timestamp_mode = pnh.param("timestamp_mode", std::string{});
        double ptp_utc_tai_offset = pnh.param("ptp_utc_tai_offset", -37.0);

        if (impl::check_token(tokens, "IMU")) {
            imu_packet_handler = ImuPacketHandler::create(
                info, tf_bcast.imu_frame_id(), timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
        }

        auto min_scan_valid_columns_ratio = pnh.param("min_scan_valid_columns_ratio", 0.0f);
        if (min_scan_valid_columns_ratio < 0.0f || min_scan_valid_columns_ratio > 1.0f) {
            NODELET_FATAL("min_scan_valid_columns_ratio needs to be in the range [0, 1]");
            throw std::runtime_error("min_scan_valid_columns_ratio out of bounds!");
        }

        std::vector<LidarScanProcessor> processors;

        if (impl::check_token(tokens, "PCL")) {
            auto point_type = pnh.param("point_type", std::string{"original"});
            auto organized = pnh.param("organized", true);
            auto destagger = pnh.param("destagger", true);
            auto min_range_m = pnh.param("min_range", 0.0);
            auto max_range_m = pnh.param("max_range", 10000.0);
            if (min_range_m < 0.0 || max_range_m < 0.0) {
                NODELET_FATAL("min_range and max_range need to be positive");
                throw std::runtime_error("negative range limits!");
            }
            if (min_range_m >= max_range_m) {
                const auto error_msg =
                    "min_range can't be equal or exceed max_range";
                NODELET_FATAL(error_msg);
                throw std::runtime_error(error_msg);
            }
            // convert to millimeters
            uint32_t min_range = impl::ulround(min_range_m * 1000);
            uint32_t max_range = impl::ulround(max_range_m * 1000);
            auto v_reduction = pnh.param("v_reduction", 1);
            auto valid_values = std::vector<int>{1, 2, 4, 8, 16};
            if (std::find(valid_values.begin(), valid_values.end(), v_reduction) == valid_values.end()) {
                NODELET_FATAL("v_reduction needs to be one of the values: {1, 2, 4, 8, 16}");
                throw std::runtime_error("invalid v_reduction value!");
            }

            processors.push_back(
                PointCloudProcessorFactory::create_point_cloud_processor(
                    point_type, info, tf_bcast.point_cloud_frame_id(),
                    tf_bcast.apply_lidar_to_sensor_transform(), organized,
                    destagger, min_range, max_range, v_reduction,
                    [this](PointCloudProcessor_OutputType msgs) {
                        for (size_t i = 0; i < msgs.size(); ++i) {
                            if (msgs[i]->header.stamp > last_msg_ts)
                                last_msg_ts = msgs[i]->header.stamp;
                            lidar_pubs[i].publish(*msgs[i]);
                        }
                    }));

            // warn about profile incompatibility
            if (PointCloudProcessorFactory::point_type_requires_intensity(
                    point_type) &&
                !PointCloudProcessorFactory::profile_has_intensity(
                    info.format.udp_profile_lidar)) {
                NODELET_WARN_STREAM(
                    "selected point type '"
                    << point_type
                    << "' is not compatible with the udp profile: "
                    << to_string(info.format.udp_profile_lidar));
            }
        }

        if (impl::check_token(tokens, "SCAN")) {
            // TODO: avoid this duplication in os_cloud_node
            int beams_count = static_cast<int>(get_beams_count(info));
            int scan_ring = pnh.param("scan_ring", 0);
            scan_ring = std::min(std::max(scan_ring, 0), beams_count - 1);
            if (scan_ring != pnh.param("scan_ring", 0)) {
                NODELET_WARN_STREAM(
                    "scan ring is set to a value that exceeds available range "
                    "please choose a value between "
                    << " [0, " << beams_count << "],"
                    << " ring value clamped to: " << scan_ring);
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

        if (impl::check_token(tokens, "PCL") ||
            impl::check_token(tokens, "SCAN")) {
            lidar_packet_handler = LidarPacketHandler::create(
                info, processors, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9),
                min_scan_valid_columns_ratio);
        }

        if (impl::check_token(tokens, "TLM")) {
            telemetry_handler = TelemetryHandler::create(
                info, timestamp_mode,
                static_cast<int64_t>(ptp_utc_tai_offset * 1e+9));
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

    ros::Publisher telemetry_pub;
    TelemetryHandler::HandlerType telemetry_handler;
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterCloud, nodelet::Nodelet)
