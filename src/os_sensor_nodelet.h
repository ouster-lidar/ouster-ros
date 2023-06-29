/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_nodelet.h
 * @brief A nodelet that connects to a live ouster sensor
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pluginlib/class_list_macros.h>

#include <fstream>
#include <string>
#include <tuple>

#include "ouster_ros/GetConfig.h"
#include "ouster_ros/SetConfig.h"
#include "ouster_ros/PacketMsg.h"
#include "ouster_ros/os_sensor_nodelet_base.h"

namespace sensor = ouster::sensor;
using nonstd::optional;
using ouster_ros::GetConfig;
using ouster_ros::SetConfig;
using ouster_ros::PacketMsg;

namespace nodelets_os {

class OusterSensor : public OusterSensorNodeletBase {
   private:
    virtual void onInit() override;

    std::string get_sensor_hostname(ros::NodeHandle& nh);

    bool update_config_and_metadata(sensor::client& cli);

    void save_metadata(ros::NodeHandle& nh);

    void create_get_config_service(ros::NodeHandle& nh);

    void create_set_config_service(ros::NodeHandle& nh);

    std::shared_ptr<sensor::client> create_sensor_client(
        const std::string& hostname, const sensor::sensor_config& config);

    std::pair<sensor::sensor_config, uint8_t> create_sensor_config_rosparams(
        ros::NodeHandle& nh);

    void configure_sensor(const std::string& hostname,
                          sensor::sensor_config& config, int config_flags);

    bool load_config_file(const std::string& config_file,
                          sensor::sensor_config& out_config);

   private:
    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode);

    void allocate_buffers();

    void create_publishers(ros::NodeHandle& nh);

    void start_connection_loop(ros::NodeHandle& nh);

    void connection_loop(sensor::client& cli, const sensor::packet_format& pf);

   private:
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    ros::Publisher lidar_packet_pub;
    ros::Publisher imu_packet_pub;
    std::shared_ptr<sensor::client> sensor_client;
    ros::Timer timer_;
    std::string sensor_hostname;
    ros::ServiceServer get_config_srv;
    ros::ServiceServer set_config_srv;
    std::string cached_config;
    std::string mtp_dest;
    bool mtp_main;
};

}  // namespace nodelets_os

// PLUGINLIB_EXPORT_CLASS(nodelets_os::OusterSensor, nodelet::Nodelet)
