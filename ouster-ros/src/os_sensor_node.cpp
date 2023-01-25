/**
 * Copyright (c) 2018-2022, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_nodelet.cpp
 * @brief A nodelet that connects to a live ouster sensor
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <fstream>
#include <string>
#include <tuple>

#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_ros/visibility_control.h"
#include "ouster_ros/srv/get_config.hpp"
#include "ouster_ros/srv/set_config.hpp"
#include "ouster_ros/os_client_base_node.h"

namespace sensor = ouster::sensor;
using nonstd::optional;
using ouster_msgs::msg::PacketMsg;
using ouster_ros::srv::GetConfig;
using ouster_ros::srv::SetConfig;

namespace nodelets_os {

class OusterSensor : public OusterClientBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterSensor(const rclcpp::NodeOptions & options)
    : OusterClientBase("os_sensor", options) {
        onInit();
    }

   private:
    virtual void onInit() override {
        declare_parameters();
        sensor_hostname = get_sensor_hostname();
        sensor::sensor_config config;
        uint8_t flags;
        std::tie(config, flags) = create_sensor_config_rosparams();
        configure_sensor(sensor_hostname, config, flags);
        sensor_client = create_sensor_client(sensor_hostname, config);
        update_config_and_metadata(*sensor_client);
        save_metadata();
        OusterClientBase::onInit();
        create_get_config_service();
        create_set_config_service();
        start_connection_loop();
    }

    void declare_parameters() {
        declare_parameter("sensor_hostname", rclcpp::PARAMETER_STRING);
        declare_parameter("metadata", rclcpp::PARAMETER_STRING);
        declare_parameter("udp_dest", rclcpp::PARAMETER_STRING);
        declare_parameter("lidar_port", 0);
        declare_parameter("imu_port", 0);
        declare_parameter("lidar_mode", rclcpp::PARAMETER_STRING);
        declare_parameter("timestamp_mode", rclcpp::PARAMETER_STRING);
        declare_parameter("udp_profile_lidar", rclcpp::PARAMETER_STRING);
    }

    std::string get_sensor_hostname() {
        auto hostname = get_parameter("sensor_hostname").as_string();
        if (!is_arg_set(hostname)) {
            auto error_msg = "Must specify a sensor hostname";
            RCLCPP_ERROR_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }

        return hostname;
    }

    bool update_config_and_metadata(sensor::client& cli) {
        sensor::sensor_config config;
        auto success = get_config(sensor_hostname, config);
        if (!success) {
            RCLCPP_ERROR(get_logger(), "Failed to collect sensor config");
            cached_config.clear();
            cached_metadata.clear();
            return false;
        }

        cached_config = to_string(config);

        try {
            cached_metadata = sensor::get_metadata(cli);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "sensor::get_metadata exception: " << e.what());
            cached_metadata.clear();
        }

        if (cached_metadata.empty()) {
            RCLCPP_ERROR(get_logger(), "Failed to collect sensor metadata");
            return false;
        }

        info = sensor::parse_metadata(cached_metadata);
        // TODO: revist when *min_version* is changed
        populate_metadata_defaults(info, sensor::MODE_UNSPEC);
        display_lidar_info(info);

        return cached_config.size() > 0 && cached_metadata.size() > 0;
    }

    void save_metadata() {
        auto meta_file = get_parameter("metadata").as_string();
        if (!is_arg_set(meta_file)) {
            meta_file = sensor_hostname.substr(0, sensor_hostname.rfind('.')) +
                        "-metadata.json";
            RCLCPP_INFO_STREAM(get_logger(),
                "No metadata file was specified, using: " << meta_file);
        }

        // write metadata file. If metadata_path is relative, will use cwd
        // (usually ~/.ros)
        if (!write_metadata(meta_file, cached_metadata)) {
            RCLCPP_ERROR(get_logger(),
                "Exiting because of failure to write metadata path");
            throw std::runtime_error("Failure to write metadata path");
        }
    }

    void create_get_config_service() {
        get_config_srv = create_service<GetConfig>("get_config",
            [this](
                const std::shared_ptr<GetConfig::Request>,
                std::shared_ptr<GetConfig::Response> response) {
                response->config = cached_config;
                return cached_config.size() > 0;
            });

        RCLCPP_INFO(get_logger(), "get_config service created");
    }

    void create_set_config_service() {
        set_config_srv = create_service<SetConfig>("set_config",
            [this](
                const std::shared_ptr<SetConfig::Request> request,
                std::shared_ptr<SetConfig::Response> response) {
                sensor::sensor_config config;
                response->config = "";
                auto success =
                    load_config_file(request->config_file, config);
                if (!success) {
                    RCLCPP_ERROR_STREAM(get_logger(),
                        "Failed to load and parse file: "
                        << request->config_file);
                    return false;
                }

                try {
                    configure_sensor(sensor_hostname, config, 0);
                } catch (const std::exception& e) {
                    return false;
                }
                success = update_config_and_metadata(*sensor_client);
                response->config = cached_config;
                return success;
            });

        RCLCPP_INFO(get_logger(), "set_config service created");
    }

    std::shared_ptr<sensor::client> create_sensor_client(
        const std::string& hostname, const sensor::sensor_config& config) {
        RCLCPP_INFO_STREAM(get_logger(),
            "Starting sensor " << hostname << " initialization...");

        int lidar_port =
            config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
        int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;

        std::shared_ptr<sensor::client> cli;
        if (lidar_port != 0 && imu_port != 0) {
            // use no-config version of init_client to bind to pre-configured
            // ports
            cli = sensor::init_client(hostname, lidar_port, imu_port);
        } else {
            // use the full init_client to generate and assign random ports to
            // sensor
            auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";
            cli = sensor::init_client(hostname, udp_dest, sensor::MODE_UNSPEC,
                                      sensor::TIME_FROM_UNSPEC, lidar_port,
                                      imu_port);
        }

        if (!cli) {
            auto error_msg = "Failed to initialize client";
            RCLCPP_ERROR_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }

        return cli;
    }

    std::pair<sensor::sensor_config, uint8_t> create_sensor_config_rosparams() {
        auto udp_dest = get_parameter("udp_dest").as_string();
        auto lidar_port = get_parameter("lidar_port").as_int();
        auto imu_port = get_parameter("imu_port").as_int();
        auto lidar_mode_arg = get_parameter("lidar_mode").as_string();
        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        auto udp_profile_lidar_arg = get_parameter("udp_profile_lidar").as_string();

        if (lidar_port < 0 || lidar_port > 65535) {
            auto error_msg =
                "Invalid lidar port number! port value should be in the range "
                "[0, 65535].";
            RCLCPP_ERROR_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }

        if (imu_port < 0 || imu_port > 65535) {
            auto error_msg =
                "Invalid imu port number! port value should be in the range "
                "[0, 65535].";
            RCLCPP_ERROR_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }

        optional<sensor::UDPProfileLidar> udp_profile_lidar;
        if (is_arg_set(udp_profile_lidar_arg)) {
            // set lidar profile from param
            udp_profile_lidar =
                sensor::udp_profile_lidar_of_string(udp_profile_lidar_arg);
            if (!udp_profile_lidar) {
                auto error_msg =
                    "Invalid udp profile lidar: " + udp_profile_lidar_arg;
                RCLCPP_ERROR_STREAM(get_logger(), error_msg);
                throw std::runtime_error(error_msg);
            }
        }

        // set lidar mode from param
        sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
        if (is_arg_set(lidar_mode_arg)) {
            lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
            if (!lidar_mode) {
                auto error_msg = "Invalid lidar mode: " + lidar_mode_arg;
                RCLCPP_ERROR_STREAM(get_logger(), error_msg);
                throw std::runtime_error(error_msg);
            }
        }

        // set timestamp mode from param
        sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
        if (is_arg_set(timestamp_mode_arg)) {
            // In case the option TIME_FROM_ROS_TIME is set then leave the
            // sensor timestamp_mode unmodified
            if (timestamp_mode_arg == "TIME_FROM_ROS_TIME") {
                RCLCPP_INFO(get_logger(),
                    "TIME_FROM_ROS_TIME timestamp mode specified."
                    " IMU and pointcloud messages will use ros time");
            } else {
                timestamp_mode =
                    sensor::timestamp_mode_of_string(timestamp_mode_arg);
                if (!timestamp_mode) {
                    auto error_msg =
                        "Invalid timestamp mode: " + timestamp_mode_arg;
                    RCLCPP_ERROR_STREAM(get_logger(), error_msg);
                    throw std::runtime_error(error_msg);
                }
            }
        }

        sensor::sensor_config config;
        if (lidar_port == 0) {
            RCLCPP_WARN(get_logger(),
                "lidar port set to zero, the client will assign a random port "
                "number!");
        } else {
            config.udp_port_lidar = lidar_port;
        }

        if (imu_port == 0) {
            RCLCPP_WARN(get_logger(),
                "imu port set to zero, the client will assign a random port "
                "number!");
        } else {
            config.udp_port_imu = imu_port;
        }

        config.udp_profile_lidar = udp_profile_lidar;
        config.operating_mode = sensor::OPERATING_NORMAL;
        if (lidar_mode) config.ld_mode = lidar_mode;
        if (timestamp_mode) config.ts_mode = timestamp_mode;

        uint8_t config_flags = 0;

        if (is_arg_set(udp_dest)) {
            RCLCPP_INFO(get_logger(),
                "Will send UDP data to %s", udp_dest.c_str());
            config.udp_dest = udp_dest;
        } else {
            RCLCPP_INFO(get_logger(),
                "Will use automatic UDP destination");
            config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
        }

        return std::make_pair(config, config_flags);
    }

    void configure_sensor(const std::string& hostname,
                          const sensor::sensor_config& config,
                          int config_flags) {
        try {
            if (!set_config(hostname, config, config_flags)) {
                auto err_msg = "Error connecting to sensor " + hostname;
                RCLCPP_ERROR_STREAM(get_logger(), err_msg);
                throw std::runtime_error(err_msg);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(),
                "Error setting config:  %s", e.what());
            throw;
        }

        RCLCPP_INFO_STREAM(get_logger(),
            "Sensor " << hostname << " configured successfully");
    }

    bool load_config_file(const std::string& config_file,
                          sensor::sensor_config& out_config) {
        std::ifstream ifs{};
        ifs.open(config_file);
        if (ifs.fail()) return false;
        std::stringstream buf;
        buf << ifs.rdbuf();
        out_config = sensor::parse_config(buf.str());
        return true;
    }

   private:
    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode) {
        if (!info.name.size()) info.name = "UNKNOWN";

        if (!info.sn.size()) info.sn = "UNKNOWN";

        ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
        if (v == ouster::util::invalid_version)
            RCLCPP_WARN(get_logger(),
                "Unknown sensor firmware version; output may not be reliable");
        else if (v < sensor::min_version)
            RCLCPP_WARN(get_logger(),
                "Firmware < %s not supported; output may not be reliable",
                to_string(sensor::min_version).c_str());

        if (!info.mode) {
            RCLCPP_WARN(get_logger(),
                "Lidar mode not found in metadata; output may not be reliable");
            info.mode = specified_lidar_mode;
        }

        if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

        if (info.beam_azimuth_angles.empty() ||
            info.beam_altitude_angles.empty()) {
            RCLCPP_ERROR(get_logger(),
                "Beam angles not found in metadata; using design values");
            info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
            info.beam_altitude_angles = sensor::gen1_altitude_angles;
        }
    }

    // try to write metadata file
    bool write_metadata(const std::string& meta_file,
                        const std::string& metadata) {
        std::ofstream ofs;
        ofs.open(meta_file);
        ofs << metadata << std::endl;
        ofs.close();
        if (ofs) {
            RCLCPP_INFO(get_logger(),
                "Wrote metadata to %s", meta_file.c_str());
        } else {
            RCLCPP_WARN(get_logger(),
                "Failed to write metadata to %s; check that the path is valid. "
                "If you provided a relative path, please note that the working "
                "directory of all ROS nodes is set by default to $ROS_HOME",
                meta_file.c_str());
            return false;
        }
        return true;
    }

    void start_connection_loop() {
        auto pf = sensor::get_format(info);
        lidar_packet.buf.resize(pf.lidar_packet_size + 1);
        imu_packet.buf.resize(pf.imu_packet_size + 1);

        lidar_packet_pub = create_publisher<PacketMsg>("lidar_packets", 1280);
        imu_packet_pub = create_publisher<PacketMsg>("imu_packets", 100);

        timer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration(0),
            std::bind(&OusterSensor::timer_callback, this));
        
    }

    void connection_loop(sensor::client& cli, const sensor::sensor_info& info) {
        auto pf = sensor::get_format(info);

        auto state = sensor::poll_client(cli);
        if (state == sensor::EXIT) {
            RCLCPP_INFO(get_logger(), "poll_client: caught signal, exiting");
            return;
        }
        if (state & sensor::CLIENT_ERROR) {
            RCLCPP_ERROR(get_logger(), "poll_client: returned error");
            return;
        }
        if (state & sensor::LIDAR_DATA) {
            if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf))
                lidar_packet_pub->publish(lidar_packet);
        }
        if (state & sensor::IMU_DATA) {
            if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf))
                imu_packet_pub->publish(imu_packet);
        }
    }

    void timer_callback() {
        connection_loop(*sensor_client, info);
    }

   private:
    std::string sensor_hostname;
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    rclcpp::Publisher<PacketMsg>::SharedPtr lidar_packet_pub;
    rclcpp::Publisher<PacketMsg>::SharedPtr imu_packet_pub;
    std::shared_ptr<sensor::client> sensor_client;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<GetConfig>::SharedPtr get_config_srv;
    rclcpp::Service<SetConfig>::SharedPtr set_config_srv;
    std::string cached_config;
};

}  // namespace nodelets_os

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(nodelets_os::OusterSensor)
