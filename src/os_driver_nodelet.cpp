/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_driver_nodelet.cpp
 * @brief This node combines the capabilities of os_sensor, os_cloud and os_img
 * into a single ROS nodelet
 */

// prevent clang-format from altering the location of "ouster_ros/os_ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "os_sensor_nodelet.h"
#include "os_transforms_broadcaster.h"
#include "imu_packet_handler.h"
#include "lidar_packet_handler.h"
#include "point_cloud_processor.h"
#include "laser_scan_processor.h"
#include "image_processor.h"
#include "point_cloud_processor_factory.h"
#include "telemetry_handler.h"

namespace sensor = ouster::sensor;
using ouster::sensor::ImuPacket;
using ouster::sensor::LidarPacket;

namespace ouster_ros {

class OusterDriver : public OusterSensor {
   public:
    OusterDriver() : tf_bcast(getName()) {}
    ~OusterDriver() override {
        NODELET_DEBUG("OusterDriver::~OusterDriver() called");
    }

   protected:
    virtual void onInit() override {
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask =
            pnh.param("proc_mask", std::string{"IMU|PCL|SCAN|IMG|RAW"});
        auto tokens = impl::parse_tokens(proc_mask, '|');
        if (impl::check_token(tokens, "IMU")) create_imu_pub();
        if (impl::check_token(tokens, "PCL")) create_point_cloud_pubs();
        if (impl::check_token(tokens, "SCAN")) create_laser_scan_pubs();
        if (impl::check_token(tokens, "IMG")) create_image_pubs();
        if (impl::check_token(tokens, "TLM")) create_telemetry_pub();
        publish_raw = impl::check_token(tokens, "RAW");
        OusterSensor::onInit();
    }

   private:
    void on_metadata_updated(const sensor::sensor_info& info) override {
        // for OusterDriver we are going to always assume static broadcast
        // at least for now
        tf_bcast.parse_parameters(getPrivateNodeHandle());
        if (tf_bcast.publish_static_tf()) {
            tf_bcast.broadcast_transforms(info);
        }
        create_handlers();
    }

    void create_imu_pub() {
        imu_pub = getNodeHandle().advertise<sensor_msgs::Imu>("imu", 100);
    }

    void create_telemetry_pub() {
        telemetry_pub =
            getNodeHandle().advertise<ouster_ros::Telemetry>("telemetry", 1280);
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

    void create_image_pubs() {
        // NOTE: always create the 2nd topics
        const std::map<sensor::ChanField, std::string> channel_field_topic_map{
            {sensor::ChanField::RANGE, "range_image"},
            {sensor::ChanField::SIGNAL, "signal_image"},
            {sensor::ChanField::REFLECTIVITY, "reflec_image"},
            {sensor::ChanField::NEAR_IR, "nearir_image"},
            {sensor::ChanField::RANGE2, "range_image2"},
            {sensor::ChanField::SIGNAL2, "signal_image2"},
            {sensor::ChanField::REFLECTIVITY2, "reflec_image2"}};

        for (auto it : channel_field_topic_map) {
            image_pubs[it.first] =
                getNodeHandle().advertise<sensor_msgs::Image>(it.second, 100);
        }
    }

    virtual void create_handlers() {
        auto& pnh = getPrivateNodeHandle();
        auto proc_mask =
            pnh.param("proc_mask", std::string{"IMU|IMG|PCL|SCAN"});
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
            if (std::find(valid_values.begin(), valid_values.end(),
                          v_reduction) == valid_values.end()) {
                NODELET_FATAL("v_reduction needs to be one of the values: {1, 2, 4, 8, 16}");
                throw std::runtime_error("invalid v_reduction value!");
            }

            processors.push_back(
                PointCloudProcessorFactory::create_point_cloud_processor(
                    point_type, info, tf_bcast.point_cloud_frame_id(),
                    tf_bcast.apply_lidar_to_sensor_transform(), organized,
                    destagger, min_range, max_range, v_reduction,
                    [this](PointCloudProcessor_OutputType msgs) {
                        for (size_t i = 0; i < msgs.size(); ++i)
                            lidar_pubs[i].publish(*msgs[i]);
                    }));

            // warn about profile incompatibility
            if (PointCloudProcessorFactory::point_type_requires_intensity(
                    point_type) &&
                !PointCloudProcessorFactory::profile_has_intensity(
                    info.format.udp_profile_lidar)) {
                NODELET_WARN_STREAM(
                    "selected point type '" << point_type
                    << "' is not compatible with the udp profile: "
                    << to_string(info.format.udp_profile_lidar));
            }
        }

        if (impl::check_token(tokens, "SCAN")) {
            // TODO: avoid duplication in os_cloud_node
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
                        scan_pubs[i].publish(*msgs[i]);
                    }
                }));
        }

        if (impl::check_token(tokens, "IMG")) {
            processors.push_back(ImageProcessor::create(
                info, tf_bcast.point_cloud_frame_id(),
                [this](ImageProcessor::OutputType msgs) {
                    for (auto it = msgs.begin(); it != msgs.end(); ++it) {
                        image_pubs[it->first].publish(*it->second);
                    }
                }));
        }

        if (impl::check_token(tokens, "PCL") ||
            impl::check_token(tokens, "SCAN") ||
            impl::check_token(tokens, "IMG")) {
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

    virtual void on_lidar_packet_msg(const LidarPacket& lidar_packet) override {
        if (telemetry_handler) {
            auto telemetry = telemetry_handler(lidar_packet);
            telemetry_pub.publish(telemetry);
        }

        if (lidar_packet_handler) {
            lidar_packet_handler(lidar_packet);
        }

        if (publish_raw) OusterSensor::on_lidar_packet_msg(lidar_packet);
    }

    virtual void on_imu_packet_msg(const ImuPacket& imu_packet) override {
        if (imu_packet_handler) {
            auto imu_msg = imu_packet_handler(imu_packet);
            imu_pub.publish(imu_msg);
        }

        if (publish_raw) OusterSensor::on_imu_packet_msg(imu_packet);
    }

   private:
    ros::Publisher imu_pub;
    std::vector<ros::Publisher> lidar_pubs;
    std::vector<ros::Publisher> scan_pubs;
    std::map<sensor::ChanField, ros::Publisher> image_pubs;

    OusterTransformsBroadcaster tf_bcast;

    ImuPacketHandler::HandlerType imu_packet_handler;
    LidarPacketHandler::HandlerType lidar_packet_handler;

    bool publish_raw = false;

    ros::Publisher telemetry_pub;
    TelemetryHandler::HandlerType telemetry_handler;
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterDriver, nodelet::Nodelet)
