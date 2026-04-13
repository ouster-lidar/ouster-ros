/**
 * Copyright (c) 2018-2023, Ouster, Inc.
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

#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

#include <chrono>

#include <ouster/metadata.h>

#include "ouster_ros/PacketMsg.h"
#include "os_sensor_nodelet.h"

using std::to_string;
using namespace std::chrono_literals;
using namespace std::string_literals;

using ouster::sdk::core::ImuPacket;
using ouster::sdk::core::LidarPacket;
using ouster::sdk::core::UDPProfileLidar;
using ouster::sdk::core::UDPProfileIMU;
using ouster::sdk::core::LidarMode;
using ouster::sdk::core::TimestampMode;
using ouster::sdk::core::OperatingMode;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::PacketFormat;
using ouster::sdk::core::SensorConfig;

namespace ouster_ros {

OusterSensor::~OusterSensor() {
    NODELET_DEBUG("OusterDriver::~OusterSensor() called");
    stop_sensor_connection_thread();
}

bool OusterSensor::start() {
    sensor_hostname = get_sensor_hostname();

    SensorConfig config;
    if (staged_config) {
        if (!configure_sensor(sensor_hostname, staged_config.value()))
            return false;
        config = staged_config.value();
        staged_config.reset();
    } else {
        if (!get_active_config_no_throw(sensor_hostname, config)) return false;

        NODELET_INFO("Retrived sensor active config");

        // Unfortunately it seems we need to invoke this to force the auto
        // TODO[UN]: find a shortcut
        // Only reset udp_dest if auto_udp was allowed on startup
        if (auto_udp_allowed) config.udp_dest.reset();
        if (!configure_sensor(sensor_hostname, config)) return false;
    }

    reset_last_init_id = true;
    sensor_client = create_sensor_client(sensor_hostname, config);
    if (!sensor_client) {
        NODELET_ERROR_STREAM("Failed to initialize client");
        return false;
    }

    update_metadata(*sensor_client);
    allocate_buffers();
    start_sensor_connection_thread();
    return true;
}

void OusterSensor::stop() {
    // deactivate
    stop_sensor_connection_thread();
    sensor_client.reset();
}

void OusterSensor::attempt_start() {
    if (!start()) {
        if (attempt_reconnect && reconnect_attempts_available-- > 0) {
            reconnect_timer = getNodeHandle().createTimer(
                ros::Duration(dormant_period_between_reconnects),
                [this](const ros::TimerEvent&) {
                    NODELET_INFO_STREAM(
                        "Attempting to communicate with the sensor, "
                        "remaining attempts: "
                        << reconnect_attempts_available);
                    attempt_start();
                },
                true);
        }
    } else {
        // reset counter
        reconnect_attempts_available = getPrivateNodeHandle().param(
            "max_failed_reconnect_attempts", INT_MAX);
    }
}

void OusterSensor::schedule_stop() {
    sensor_connection_active = false;
    reconnect_timer = getNodeHandle().createTimer(
        ros::Duration(0.0),
        [this](const ros::TimerEvent&) {
            stop();
            if (attempt_reconnect && reconnect_attempts_available-- > 0) {
                NODELET_INFO_STREAM(
                    "Attempting to communicate with the sensor, "
                    "remaining attempts: "
                    << reconnect_attempts_available);
                attempt_start();
            }
        },
        true);
}

void OusterSensor::onInit() {
    staged_config = parse_config_from_ros_parameters();
    create_metadata_pub();
    create_services();
    create_publishers();
    attempt_reconnect =
        getPrivateNodeHandle().param("attempt_reconnect", false);
    dormant_period_between_reconnects =
        getPrivateNodeHandle().param("dormant_period_between_reconnects", 1.0);
    reconnect_attempts_available =
        getPrivateNodeHandle().param("max_failed_reconnect_attempts", INT_MAX);
    attempt_start();
}

std::string OusterSensor::get_sensor_hostname() {
    auto& nh = getPrivateNodeHandle();
    auto hostname = nh.param("sensor_hostname", std::string{});
    if (!is_arg_set(hostname)) {
        auto error_msg = "Must specify a sensor hostname";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    return hostname;
}

void OusterSensor::update_metadata(ouster::sdk::sensor::Client& cli) {
    try {
        cached_metadata = ouster::sdk::sensor::get_metadata(cli, 60);
    } catch (const std::exception& e) {
        NODELET_ERROR_STREAM("ouster::sdk::sensor::get_metadata exception: " << e.what());
        cached_metadata.clear();
    }

    if (cached_metadata.empty()) {
        const auto error_msg = "Failed to collect sensor metadata";
        NODELET_ERROR(error_msg);
        throw std::runtime_error(error_msg);
    }

    info = ouster::sdk::core::SensorInfo(cached_metadata);
    // TODO: revist when *min_version* is changed
    populate_metadata_defaults(info, LidarMode::UNSPECIFIED);

    publish_metadata();
    save_metadata();
    metadata_updated(info);
}

void OusterSensor::save_metadata() {
    auto& nh = getPrivateNodeHandle();
    auto meta_file = nh.param("metadata", std::string{});
    if (!is_arg_set(meta_file)) {
        meta_file = sensor_hostname.substr(0, sensor_hostname.rfind('.')) +
                    "-metadata.json";
        NODELET_INFO_STREAM(
            "No metadata file was specified, using: " << meta_file);
    }

    // write metadata file. If metadata_path is relative, will use cwd
    // (usually ~/.ros)
    if (write_text_to_file(meta_file, cached_metadata)) {
        NODELET_INFO_STREAM("Wrote sensor metadata to " << meta_file);
    } else {
        NODELET_WARN_STREAM(
            "Failed to write metadata to " << meta_file
            << "; check that the path is valid. If you provided a relative "
               "path, please note that the working directory of all ROS "
               "nodes is set by default to $ROS_HOME");
    }
}

void OusterSensor::create_reset_service() {
    reset_srv = getNodeHandle()
                    .advertiseService<std_srvs::Empty::Request,
                                      std_srvs::Empty::Response>(
                        "reset", [this](std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
                            NODELET_INFO("reset service invoked");
                            reset_sensor(true);
                            return true;
                        });

    NODELET_INFO("reset service created");
}

bool OusterSensor::get_active_config_no_throw(
    const std::string& sensor_hostname, SensorConfig& config) {
    try {
        if (ouster::sdk::sensor::get_config(sensor_hostname, config, true)) return true;
    } catch (const std::exception&) {
        NODELET_ERROR_STREAM(
            "Couldn't get active config for: " << sensor_hostname);
        return false;
    }

    NODELET_ERROR_STREAM("Couldn't get active config for: " << sensor_hostname);
    return false;
}

void OusterSensor::create_get_config_service() {
    get_config_srv =
        getNodeHandle()
            .advertiseService<GetConfig::Request, GetConfig::Response>(
                "get_config",
                [this](GetConfig::Request&, GetConfig::Response& response) {
                    std::string active_config;
                    SensorConfig config;
                    if (get_active_config_no_throw(sensor_hostname, config))
                        active_config = to_string(config);
                    response.config = active_config;
                    return active_config.size() > 0;
                });

    NODELET_INFO("get_config service created");
}

void OusterSensor::create_set_config_service() {
    set_config_srv =
        getNodeHandle()
            .advertiseService<SetConfig::Request, SetConfig::Response>(
                "set_config", [this](SetConfig::Request& request,
                                     SetConfig::Response& response) {
                    response.config = "";

                    std::string config_str;
                    try {
                        config_str = read_text_file(request.config_file);
                        if (config_str.empty()) {
                            NODELET_ERROR_STREAM(
                                "provided config file: "
                                << request.config_file
                                << " turned to be empty. set_config ignored!");
                            return false;
                        }
                    } catch (const std::exception& e) {
                        NODELET_ERROR_STREAM(
                            "exception thrown while loading config file: "
                            << request.config_file
                            << ", exception details: " << e.what());
                        return false;
                    }

                    SensorConfig config;
                    if (!ouster::sdk::core::parse_and_validate_config(config_str, config)) {
                        return false;
                    }
                    staged_config = config;
                    response.config = config_str;
                    // TODO: this is currently set to force_reinit but it
                    // doesn't need to be the case if it was possible to know
                    // that the new config would result in a reinit when a
                    // reinit is not forced
                    reset_sensor(true);
                    return true;
                });

    NODELET_INFO("set_config service created");
}

std::shared_ptr<ouster::sdk::sensor::Client> OusterSensor::create_sensor_client(
    const std::string& hostname, const SensorConfig& config) {
    int lidar_port = config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
    int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;
    auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";

    NODELET_INFO_STREAM("Starting sensor " << hostname
                        << " initialization... Using ports: "
                        << lidar_port << "/" << imu_port);

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

    std::shared_ptr<ouster::sdk::sensor::Client> cli;
    if (ouster::sdk::sensor::in_multicast(udp_dest)) {
        // use the mtp_init_client to recieve data via multicast
        // if mtp_main is true when sensor will be configured
            cli = ouster::sdk::sensor::mtp_init_client(hostname, config, mtp_dest, mtp_main);
    } else if (lidar_port != 0 && imu_port != 0) {
        // use no-config version of init_client to bind to pre-configured
        // ports
        cli = ouster::sdk::sensor::init_client(hostname, lidar_port, imu_port);
    } else {
        // use the full init_client to generate and assign random ports to
        // sensor
        cli = ouster::sdk::sensor::init_client(hostname, udp_dest, LidarMode::UNSPECIFIED,
                                               TimestampMode::UNSPECIFIED, lidar_port, imu_port);
    }

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    return cli;
}

void OusterSensor::parse_udp_dest_and_ports(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();

    auto udp_dest = nh.param("udp_dest", std::string{});
    if (!is_arg_set(udp_dest))
        udp_dest = nh.param("computer_ip", std::string{});

    auto mtp_dest_arg = nh.param("mtp_dest", std::string{});
    auto mtp_main_arg = nh.param("mtp_main", false);

    if (is_arg_set(udp_dest)) {
        config.udp_dest = udp_dest;
        if (ouster::sdk::sensor::in_multicast(udp_dest)) {
            mtp_dest = is_arg_set(mtp_dest_arg) ? mtp_dest_arg : std::string{};
            mtp_main = mtp_main_arg;
        }
    } else {
        auto_udp_allowed = true;
    }

    // parse lidar port
    auto lidar_port = nh.param("lidar_port", 0);
    if (lidar_port < 0 || lidar_port > 65535) {
        auto error_msg =
            "Invalid lidar port number! port value should be in the range "
            "[0, 65535].";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    if (lidar_port == 0) {
        NODELET_WARN_COND(
            !is_arg_set(mtp_dest_arg),
            "lidar port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_lidar = lidar_port;
    }

    // parse imu port
    auto imu_port = nh.param("imu_port", 0);
    if (imu_port < 0 || imu_port > 65535) {
        auto error_msg =
            "Invalid imu port number! port value should be in the range "
            "[0, 65535].";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    if (imu_port == 0) {
        NODELET_WARN_COND(
            !is_arg_set(mtp_dest_arg),
            "imu port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_imu = imu_port;
    }
}

void OusterSensor::parse_udp_profile_lidar(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto udp_profile_lidar_arg = nh.param("udp_profile_lidar", std::string{});
    if (!is_arg_set(udp_profile_lidar_arg)) {
        return;
    }

    auto udp_profile_lidar =
        ouster::sdk::core::udp_profile_lidar_of_string(udp_profile_lidar_arg);
    if (!udp_profile_lidar) {
        auto error_msg =
            "Invalid udp profile lidar: " + udp_profile_lidar_arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.udp_profile_lidar = udp_profile_lidar;
}

void OusterSensor::parse_udp_profile_imu_and_settings(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto udp_profile_imu_arg = nh.param("udp_profile_imu", std::string{});

    if (is_arg_set(udp_profile_imu_arg)) {
        auto udp_profile_imu =
            ouster::sdk::core::udp_profile_imu_of_string(udp_profile_imu_arg);
        if (!udp_profile_imu) {
            auto error_msg =
                "Invalid udp profile imu: " + udp_profile_imu_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
        config.udp_profile_imu = udp_profile_imu;
    }

    auto imu_packets_per_frame = nh.param("imu_packets_per_frame", 0);
    if (imu_packets_per_frame != 0) {
        auto valid_values = std::vector<int>{1, 2, 4, 8};
        if (std::find(valid_values.begin(), valid_values.end(),
                        imu_packets_per_frame) == valid_values.end()) {
            NODELET_FATAL(
                "imu_packets_per_frame needs to be one of the values: {1, 2, 4, 8}");
            throw std::runtime_error("invalid imu_packets_per_frame value!");
        }
        config.imu_packets_per_frame = imu_packets_per_frame;
    }

    auto gyro_fsr_arg = nh.param("gyro_fsr", std::string{});
    if (is_arg_set(gyro_fsr_arg)) {
        auto gyro_fsr = ouster::sdk::core::full_scale_range_of_string(gyro_fsr_arg);
        if (!gyro_fsr) {
            auto error_msg = "Invalid gyro fsr: " + gyro_fsr_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
        config.gyro_fsr = gyro_fsr.value();
    }

    auto accel_fsr_arg = nh.param("accel_fsr", std::string{});
    if (is_arg_set(accel_fsr_arg)) {
        auto accel_fsr = ouster::sdk::core::full_scale_range_of_string(accel_fsr_arg);
        if (!accel_fsr) {
            auto error_msg = "Invalid accel fsr: " + accel_fsr_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
        config.accel_fsr = accel_fsr.value();
    }
}

void OusterSensor::parse_multipurpose_io_mode(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto arg = nh.param("multipurpose_io_mode", std::string{});
    if (!is_arg_set(arg)) return;

    auto mode = ouster::sdk::core::multipurpose_io_mode_of_string(arg);
    if (!mode) {
        auto error_msg = "Invalid multipurpose io mode: " + arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.multipurpose_io_mode = mode.value();
}

void OusterSensor::parse_nmea_in_polarity(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto arg = nh.param("nmea_in_polarity", std::string{});
    if (!is_arg_set(arg)) return;

    auto pol = ouster::sdk::core::polarity_of_string(arg);
    if (!pol) {
        auto error_msg = "Invalid nmea_in_polarity: " + arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.nmea_in_polarity = pol.value();
}

void OusterSensor::parse_nmea_ignore_valid_char(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    config.nmea_ignore_valid_char = nh.param("nmea_ignore_valid_char", false);
}

void OusterSensor::parse_nmea_baud_rate(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto arg = nh.param("nmea_baud_rate", std::string{});
    if (!is_arg_set(arg)) return;

    auto rate = ouster::sdk::core::nmea_baud_rate_of_string(arg);
    if (!rate) {
        auto error_msg = "Invalid nmea_baud_rate: " + arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.nmea_baud_rate = rate.value();
}

void OusterSensor::parse_nmea_leap_seconds(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto val = nh.param("nmea_leap_seconds", 0);
    config.nmea_leap_seconds = val;
}

void OusterSensor::parse_sync_pulse_in_polarity(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto arg = nh.param("sync_pulse_in_polarity", std::string{});
    if (!is_arg_set(arg)) return;

    auto pol = ouster::sdk::core::polarity_of_string(arg);
    if (!pol) {
        auto error_msg = "Invalid sync_pulse_in_polarity: " + arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.sync_pulse_in_polarity = pol.value();
}

void OusterSensor::parse_sync_pulse_out_polarity(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto arg = nh.param("sync_pulse_out_polarity", std::string{});
    if (!is_arg_set(arg)) return;

    auto pol = ouster::sdk::core::polarity_of_string(arg);
    if (!pol) {
        auto error_msg = "Invalid sync_pulse_out_polarity: " + arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.sync_pulse_out_polarity = pol.value();
}

void OusterSensor::parse_sync_pulse_out_frequency(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto val = nh.param("sync_pulse_out_frequency", -1);
    if (val < 0) return;
    config.sync_pulse_out_frequency = val;
}

void OusterSensor::parse_sync_pulse_out_angle(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto val = nh.param("sync_pulse_out_angle", -1);
    if (val < 0) return;
    config.sync_pulse_out_angle = val;
}

void OusterSensor::parse_sync_pulse_out_pulse_width(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto val = nh.param("sync_pulse_out_pulse_width", -1);
    if (val < 0) return;
    config.sync_pulse_out_pulse_width = val;
}

void OusterSensor::parse_lidar_mode(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
    if (!is_arg_set(lidar_mode_arg)) {
        return;
    }

    auto lidar_mode = ouster::sdk::core::lidar_mode_of_string(lidar_mode_arg);
    if (lidar_mode == LidarMode::UNSPECIFIED) {
        auto error_msg = "Invalid lidar mode: " + lidar_mode_arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.lidar_mode = lidar_mode;
}

void OusterSensor::parse_timestamp_mode(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});
    if (!is_arg_set(timestamp_mode_arg)) {
        return;
    }

    // In case the option TIME_FROM_ROS_TIME is set then leave the
    // sensor timestamp_mode unmodified
    if (timestamp_mode_arg == "TIME_FROM_ROS_TIME" ||
        timestamp_mode_arg == "TIME_FROM_ROS_RECEPTION") {
        NODELET_INFO("TIME_FROM_ROS_TIME timestamp mode specified."
                     " IMU and pointcloud messages will use ros time");
    } else {
        auto timestamp_mode =
            ouster::sdk::core::timestamp_mode_of_string(timestamp_mode_arg);
        if (timestamp_mode == TimestampMode::UNSPECIFIED) {
            auto error_msg =
                "Invalid timestamp mode: " + timestamp_mode_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
        config.timestamp_mode = timestamp_mode;
    }
}

void OusterSensor::parse_azimuth_window(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto azimuth_window_start = nh.param("azimuth_window_start", MIN_AZW);
    auto azimuth_window_end = nh.param("azimuth_window_end", MAX_AZW);
    if (azimuth_window_start < MIN_AZW || azimuth_window_start > MAX_AZW ||
        azimuth_window_end < MIN_AZW || azimuth_window_end > MAX_AZW) {
        auto error_msg = "azimuth window values must be between " +
                    to_string(MIN_AZW) + " and " + to_string(MAX_AZW);
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    config.azimuth_window = {azimuth_window_start, azimuth_window_end};
}

void OusterSensor::parse_operating_mode(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto operating_mode_arg = nh.param("operating_mode", std::string{});
    if (!is_arg_set(operating_mode_arg)) {
        return;
    }

    auto operating_mode = ouster::sdk::core::operating_mode_of_string(operating_mode_arg);
    if (!operating_mode) {
        auto error_msg = "Invalid operating mode: " + operating_mode_arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.operating_mode = operating_mode.value();
}

void OusterSensor::parse_signal_multiplier(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto signal_multiplier = nh.param("signal_multiplier", 1.0);
    try {
        ouster::sdk::core::check_signal_multiplier(signal_multiplier);
    } catch (const std::exception& e) {
        auto error_msg = "Invalid signal multiplier: " + to_string(signal_multiplier) +
                         ", exception details: " + e.what();
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.signal_multiplier = signal_multiplier;
}

void OusterSensor::parse_phase_lock_and_offset(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    config.phase_lock_enable = nh.param("phase_lock_enable", false);
    auto phase_lock_offset = nh.param("phase_lock_offset", 0);
    if (phase_lock_offset < MIN_AZW || phase_lock_offset > MAX_AZW) {
        auto error_msg = "phase_lock_offset must be between 0 and 360000 millidegrees";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.phase_lock_offset = phase_lock_offset;
}

void OusterSensor::parse_min_distance(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto min_distance = nh.param("min_distance", -1);
    if (min_distance < 0) return;
    // Only allow exact values: 0, 30, or 50 (cm)
    if (min_distance != 0 && min_distance != 30 && min_distance != 50) {
        NODELET_FATAL("min_distance must be one of {0, 30, 50} cm");
        throw std::runtime_error("invalid min_distance value!");
    }
    config.min_range_threshold_cm = min_distance;
}

void OusterSensor::parse_lidar_frame_azimuth_offset(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto azimuth_offset = nh.param("lidar_frame_azimuth_offset", -1);
    if (azimuth_offset < 0) {
        return;
    }
    config.lidar_frame_azimuth_offset = azimuth_offset;
}

void OusterSensor::parse_return_order(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto return_order_arg = nh.param("return_order", std::string{});
    if (!is_arg_set(return_order_arg)) {
        return;
    }

    auto return_order = ouster::sdk::core::return_order_of_string(return_order_arg);
    if (!return_order) {
        auto error_msg = "Invalid return order: " + return_order_arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.return_order = return_order.value();
}

void OusterSensor::parse_bloom_reduction_optimization(SensorConfig& config) {
    auto& nh = getPrivateNodeHandle();
    auto bloom_reduction_arg = nh.param("bloom_reduction_optimization", std::string{});
    if (!is_arg_set(bloom_reduction_arg)) {
        return;
    }
    auto bloom_reduction = ouster::sdk::core::bloom_reduction_optimization_of_string(bloom_reduction_arg);
    if (!bloom_reduction) {
        auto error_msg = "Invalid bloom reduction optimization: " + bloom_reduction_arg;
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }
    config.bloom_reduction_optimization = bloom_reduction.value();
}

void OusterSensor::parse_persist_config_flag() {
    auto& nh = getPrivateNodeHandle();
    auto lidar_port = nh.param("lidar_port", 0);
    auto imu_port = nh.param("imu_port", 0);
    persist_config = nh.param("persist_config", false);
    if (persist_config && (lidar_port == 0 || imu_port == 0)) {
        NODELET_WARN("When using persist_config it is recommended "
        " to not use 0 for port values as this currently will trigger sensor "
        " reinit event each time");
    }
}

SensorConfig OusterSensor::parse_config_from_ros_parameters() {
    SensorConfig config;
    parse_udp_dest_and_ports(config);
    parse_udp_profile_lidar(config);
    parse_udp_profile_imu_and_settings(config);
    parse_lidar_mode(config);
    parse_timestamp_mode(config);
    parse_azimuth_window(config);
    parse_operating_mode(config);
    parse_signal_multiplier(config);
    parse_multipurpose_io_mode(config);
    parse_nmea_in_polarity(config);
    parse_nmea_ignore_valid_char(config);
    parse_nmea_baud_rate(config);
    parse_nmea_leap_seconds(config);
    parse_sync_pulse_in_polarity(config);
    parse_sync_pulse_out_polarity(config);
    parse_sync_pulse_out_frequency(config);
    parse_sync_pulse_out_angle(config);
    parse_sync_pulse_out_pulse_width(config);
    parse_phase_lock_and_offset(config);
    parse_min_distance(config);
    parse_lidar_frame_azimuth_offset(config);
    parse_return_order(config);
    parse_bloom_reduction_optimization(config);
    parse_persist_config_flag();
    return config;
}

uint8_t OusterSensor::compose_config_flags(
    const SensorConfig& config) {
    uint8_t config_flags = 0;
    if (config.udp_dest) {
        NODELET_INFO_STREAM("Will send UDP data to "
                            << config.udp_dest.value());
        // TODO: revise multicast setup inference
        if (ouster::sdk::sensor::in_multicast(*config.udp_dest)) {
            if (is_arg_set(mtp_dest)) {
                NODELET_INFO_STREAM("Will recieve data via multicast on "
                                    << mtp_dest);
            } else {
                NODELET_INFO(
                    "mtp_dest was not set, will recieve data via multicast "
                    "on first available interface");
            }
        }
    } else {
        NODELET_INFO("Will use automatic UDP destination");
        config_flags |= ouster::sdk::sensor::CONFIG_UDP_DEST_AUTO;
    }

    if (force_sensor_reinit) {
        force_sensor_reinit = false;
        NODELET_INFO("Forcing sensor to reinitialize");
        config_flags |= ouster::sdk::sensor::CONFIG_FORCE_REINIT;
    }

    if (persist_config) {
        persist_config =
            false;  // avoid persisting configs implicitly on restarts
        NODELET_INFO("Configuration will be persisted");
        config_flags |= ouster::sdk::sensor::CONFIG_PERSIST;
    }

    return config_flags;
}

bool OusterSensor::configure_sensor(const std::string& hostname,
                                    SensorConfig& config) {
    // TODO[UN]: in future always get_config
    if (config.udp_dest && ouster::sdk::sensor::in_multicast(config.udp_dest.value()) &&
        !mtp_main) {
        if (!ouster::sdk::sensor::get_config(hostname, config, true)) {
            NODELET_ERROR("Error getting active config");
            return false;
        }

        NODELET_INFO("Retrived active config of sensor");
        return true;
    }

    uint8_t config_flags = compose_config_flags(config);
    NODELET_INFO_STREAM("Contacting sensor " << hostname << " ...");
    try {
        ouster::sdk::sensor::set_config(hostname, config, config_flags);
    } catch (const std::exception& ex) {
        NODELET_ERROR_STREAM("Error connecting to sensor "
                             << hostname << ", details: " << ex.what());
        return false;
    }

    NODELET_INFO_STREAM("Sensor " << hostname
                                  << " was configured successfully");
    return true;
}

void OusterSensor::populate_metadata_defaults(
    SensorInfo& info, LidarMode specified_lidar_mode) {
    ouster::sdk::core::Version v = ouster::sdk::core::version_from_string(info.image_rev);
    if (v == ouster::sdk::core::INVALID_VERSION)
        NODELET_WARN(
            "Unknown sensor firmware version; output may not be reliable");
    else if (v < ouster::sdk::sensor::MIN_VERSION)
        NODELET_WARN("Firmware < %s not supported; output may not be reliable",
                     ouster::sdk::sensor::MIN_VERSION.simple_version_string().c_str());

    if (!info.config.lidar_mode) {
        NODELET_WARN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.config.lidar_mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        NODELET_ERROR("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = ouster::sdk::core::GEN1_AZIMUTH_ANGLES;
        info.beam_altitude_angles = ouster::sdk::core::GEN1_ALTITUDE_ANGLES;
    }
}

void OusterSensor::on_metadata_updated(const SensorInfo&) {}

void OusterSensor::metadata_updated(const SensorInfo& info) {
    display_lidar_info(info);
    on_metadata_updated(info);
}

void OusterSensor::create_services() {
    create_reset_service();
    create_get_metadata_service();
    create_get_config_service();
    create_set_config_service();
}

void OusterSensor::create_publishers() {
    auto& nh = getNodeHandle();
    lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
    imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);
}

void OusterSensor::allocate_buffers() {
    auto& pf = ouster::sdk::core::get_format(info);
    auto packet_format = std::make_shared<PacketFormat>(pf);
    lidar_packet.buf.resize(pf.lidar_packet_size);
    lidar_packet.format = packet_format;
    lidar_packet_msg.buf.resize(pf.lidar_packet_size);
    imu_packet.buf.resize(pf.imu_packet_size);
    imu_packet.format = packet_format;
    imu_packet_msg.buf.resize(pf.imu_packet_size);
}

bool OusterSensor::init_id_changed(const PacketFormat& pf,
                                   const LidarPacket& lidar_packet) {
    uint32_t current_init_id = pf.init_id(lidar_packet.buf.data());
    if (!last_init_id) {
        last_init_id = current_init_id + 1;
    }
    if (reset_last_init_id && last_init_id != current_init_id) {
        last_init_id = current_init_id;
        reset_last_init_id = false;
        return false;
    }
    if (last_init_id == current_init_id) return false;
    last_init_id = current_init_id;
    return true;
}

void OusterSensor::handle_poll_client_error() {
    NODELET_WARN_THROTTLE(
        1, "sensor::poll_client()) returned an error or timed out");
    // in case error continues for a while attempt to recover by
    // performing sensor reset
    if (++poll_client_error_count > max_poll_client_error_count) {
        NODELET_ERROR(
            "maximum number of allowed errors from "
            "sensor::poll_client() reached, performing self reset...");
        poll_client_error_count = 0;
        reset_sensor(true);
    }
}

void OusterSensor::read_lidar_packet(ouster::sdk::sensor::Client& cli,
                                     const PacketFormat& pf) {
    if (ouster::sdk::sensor::read_lidar_packet(cli, lidar_packet)) {
        read_lidar_packet_errors = 0;
        if (!is_legacy_lidar_profile(info) &&
            init_id_changed(pf, lidar_packet)) {
            // TODO: short circut reset if no breaking changes occured?
            NODELET_WARN("sensor init_id has changed! reactivating..");
            reset_sensor(false);
        }
        handle_lidar_packet(lidar_packet);
    } else {
        if (++read_lidar_packet_errors > max_read_lidar_packet_errors) {
            NODELET_ERROR(
                "maximum number of allowed errors from "
                "sensor::read_lidar_packet() reached, reactivating...");
            read_lidar_packet_errors = 0;
            reset_sensor(true);
        }
    }
}

void OusterSensor::handle_lidar_packet(const LidarPacket& lidar_packet) {
    on_lidar_packet_msg(lidar_packet);
}

void OusterSensor::read_imu_packet(ouster::sdk::sensor::Client& cli,
                                   const PacketFormat&) {
    if (ouster::sdk::sensor::read_imu_packet(cli, imu_packet)) {
        handle_imu_packet(imu_packet);
    } else {
        if (++read_imu_packet_errors > max_read_imu_packet_errors) {
            NODELET_ERROR(
                "maximum number of allowed errors from "
                "sensor::read_imu_packet() reached, reactivating...");
            read_imu_packet_errors = 0;
            reactivate_sensor(true);
        }
    }
}

void OusterSensor::handle_imu_packet(const ImuPacket& imu_packet) {
    on_imu_packet_msg(imu_packet);
}

void OusterSensor::connection_loop(ouster::sdk::sensor::Client& cli,
                                   const PacketFormat& pf) {
    auto state = ouster::sdk::sensor::poll_client(cli);
    if (state == ouster::sdk::sensor::EXIT) {
        NODELET_INFO("poll_client: caught signal, exiting!");
        return;
    }
    if (state & ouster::sdk::sensor::ERR || state == ouster::sdk::sensor::TIMEOUT) {
        handle_poll_client_error();
        return;
    }
    poll_client_error_count = 0;
    if (state & ouster::sdk::sensor::LIDAR_DATA) {
        read_lidar_packet(cli, pf);
    }
    if (state & ouster::sdk::sensor::IMU_DATA) {
        read_imu_packet(cli, pf);
    }
}

void OusterSensor::start_sensor_connection_thread() {
    sensor_connection_active = true;
    sensor_connection_thread = std::make_unique<std::thread>([this]() {
        NODELET_DEBUG("sensor_connection_thread active.");
        auto& pf = ouster::sdk::core::get_format(info);
        while (ros::ok() && sensor_connection_active) {
            connection_loop(*sensor_client, pf);
        }
        NODELET_DEBUG("sensor_connection_thread done.");
    });
}

void OusterSensor::stop_sensor_connection_thread() {
    NODELET_DEBUG("sensor_connection_thread stopping.");
    if ((sensor_connection_thread != nullptr) &&
        sensor_connection_thread->joinable()) {
        sensor_connection_active = false;
        sensor_connection_thread->join();
    }
}

void OusterSensor::on_lidar_packet_msg(const LidarPacket&) {
    lidar_packet_msg.buf.swap(lidar_packet.buf);
    lidar_packet_pub.publish(lidar_packet_msg);
}

void OusterSensor::on_imu_packet_msg(const ImuPacket&) {
    imu_packet_msg.buf.swap(imu_packet.buf);
    imu_packet_pub.publish(imu_packet_msg);
}

// param init_id_reset is overriden to true when force_reinit is true
void OusterSensor::reset_sensor(bool /*force_reinit*/, bool /*init_id_reset*/) {
    schedule_stop();
}

void OusterSensor::reactivate_sensor(bool /*init_id_reset*/) {
    NODELET_WARN(
        "sensor reactivate is invoked but sensor it is not implemented");
}

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterSensor, nodelet::Nodelet)
