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

#include "ouster_ros/PacketMsg.h"
#include "os_sensor_nodelet.h"

using std::to_string;
using namespace std::chrono_literals;
using namespace std::string_literals;

namespace sensor = ouster::sensor;
using sensor::ImuPacket;
using sensor::LidarPacket;

namespace ouster_ros {

OusterSensor::~OusterSensor() {
    NODELET_DEBUG("OusterDriver::~OusterSensor() called");
    stop_sensor_connection_thread();
}

bool OusterSensor::start() {
    sensor_hostname = get_sensor_hostname();

    sensor::sensor_config config;
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

void OusterSensor::update_metadata(sensor::client& cli) {
    try {
        cached_metadata = sensor::get_metadata(cli, 60, false);
    } catch (const std::exception& e) {
        NODELET_ERROR_STREAM("sensor::get_metadata exception: " << e.what());
        cached_metadata.clear();
    }

    if (cached_metadata.empty()) {
        const auto error_msg = "Failed to collect sensor metadata";
        NODELET_ERROR(error_msg);
        throw std::runtime_error(error_msg);
    }

    info = sensor::parse_metadata(cached_metadata);
    // TODO: revist when *min_version* is changed
    populate_metadata_defaults(info, sensor::MODE_UNSPEC);

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
    const std::string& sensor_hostname, sensor::sensor_config& config) {
    try {
        if (get_config(sensor_hostname, config, true)) return true;
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
                    sensor::sensor_config config;
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

                    staged_config = sensor::parse_config(config_str);
                    // TODO: this is currently set to force_reinit but it
                    // doesn't need to be the case if it was possible to know
                    // that the new config would result in a reinit when a
                    // reinit is not forced
                    reset_sensor(true);
                    response.config = config_str;
                    return true;
                });

    NODELET_INFO("set_config service created");
}

std::shared_ptr<sensor::client> OusterSensor::create_sensor_client(
    const std::string& hostname, const sensor::sensor_config& config) {
    int lidar_port = config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
    int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;
    auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";

    NODELET_INFO_STREAM("Starting sensor " << hostname
                                           << " initialization..."
                                              " Using ports: "
                                           << lidar_port << "/" << imu_port);

    std::shared_ptr<sensor::client> cli;
    if (sensor::in_multicast(udp_dest)) {
        // use the mtp_init_client to recieve data via multicast
        // if mtp_main is true when sensor will be configured
        cli = sensor::mtp_init_client(hostname, config, mtp_dest, mtp_main);
    } else if (lidar_port != 0 && imu_port != 0) {
        // use no-config version of init_client to bind to pre-configured
        // ports
        cli = sensor::init_client(hostname, lidar_port, imu_port);
    } else {
        // use the full init_client to generate and assign random ports to
        // sensor
        cli =
            sensor::init_client(hostname, udp_dest, sensor::MODE_UNSPEC,
                                sensor::TIME_FROM_UNSPEC, lidar_port, imu_port);
    }

    return cli;
}

sensor::sensor_config OusterSensor::parse_config_from_ros_parameters() {
    auto& nh = getPrivateNodeHandle();
    auto udp_dest = nh.param("udp_dest", std::string{});
    auto mtp_dest_arg = nh.param("mtp_dest", std::string{});
    auto mtp_main_arg = nh.param("mtp_main", false);
    auto lidar_port = nh.param("lidar_port", 0);
    auto imu_port = nh.param("imu_port", 0);
    auto lidar_mode_arg = nh.param("lidar_mode", std::string{});
    auto timestamp_mode_arg = nh.param("timestamp_mode", std::string{});
    auto udp_profile_lidar_arg = nh.param("udp_profile_lidar", std::string{});
    const int MIN_AZW = 0, MAX_AZW = 360000;
    auto azimuth_window_start = nh.param("azimuth_window_start", MIN_AZW);
    auto azimuth_window_end = nh.param("azimuth_window_end", MAX_AZW);

    if (lidar_port < 0 || lidar_port > 65535) {
        auto error_msg =
            "Invalid lidar port number! port value should be in the range "
            "[0, 65535].";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    if (imu_port < 0 || imu_port > 65535) {
        auto error_msg =
            "Invalid imu port number! port value should be in the range "
            "[0, 65535].";
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    nonstd::optional<sensor::UDPProfileLidar> udp_profile_lidar;
    if (is_arg_set(udp_profile_lidar_arg)) {
        // set lidar profile from param
        udp_profile_lidar =
            sensor::udp_profile_lidar_of_string(udp_profile_lidar_arg);
        if (!udp_profile_lidar) {
            auto error_msg =
                "Invalid udp profile lidar: " + udp_profile_lidar_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    // set lidar mode from param
    sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
    if (is_arg_set(lidar_mode_arg)) {
        lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
        if (!lidar_mode) {
            auto error_msg = "Invalid lidar mode: " + lidar_mode_arg;
            NODELET_FATAL_STREAM(error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    // set timestamp mode from param
    sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
    if (is_arg_set(timestamp_mode_arg)) {
        // In case the option TIME_FROM_ROS_TIME is set then leave the
        // sensor timestamp_mode unmodified
        if (timestamp_mode_arg == "TIME_FROM_ROS_TIME") {
            NODELET_INFO(
                "TIME_FROM_ROS_TIME timestamp mode specified."
                " IMU and pointcloud messages will use ros time");
        } else {
            timestamp_mode =
                sensor::timestamp_mode_of_string(timestamp_mode_arg);
            if (!timestamp_mode) {
                auto error_msg =
                    "Invalid timestamp mode: " + timestamp_mode_arg;
                NODELET_FATAL_STREAM(error_msg);
                throw std::runtime_error(error_msg);
            }
        }
    }

    sensor::sensor_config config;
    if (lidar_port == 0) {
        NODELET_WARN_COND(
            !is_arg_set(mtp_dest_arg),
            "lidar port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_lidar = lidar_port;
    }

    if (imu_port == 0) {
        NODELET_WARN_COND(
            !is_arg_set(mtp_dest_arg),
            "imu port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_imu = imu_port;
    }

    persist_config = nh.param("persist_config", false);
    if (persist_config && (lidar_port == 0 || imu_port == 0)) {
        NODELET_WARN(
            "When using persist_config it is recommended to not "
            "use 0 for port values as this currently will trigger sensor "
            "reinit "
            "event each time");
    }

    config.udp_profile_lidar = udp_profile_lidar;
    config.operating_mode = sensor::OPERATING_NORMAL;
    if (lidar_mode) config.ld_mode = lidar_mode;
    if (timestamp_mode) config.ts_mode = timestamp_mode;
    if (is_arg_set(udp_dest)) {
        config.udp_dest = udp_dest;
        if (sensor::in_multicast(udp_dest)) {
            mtp_dest = is_arg_set(mtp_dest_arg) ? mtp_dest_arg : std::string{};
            mtp_main = mtp_main_arg;
        }
    } else {
        auto_udp_allowed = true;
    }

    if (azimuth_window_start < MIN_AZW || azimuth_window_start > MAX_AZW ||
        azimuth_window_end < MIN_AZW || azimuth_window_end > MAX_AZW) {
        auto error_msg = "azimuth window values must be between " +
                         to_string(MIN_AZW) + " and " + to_string(MAX_AZW);
        NODELET_FATAL_STREAM(error_msg);
        throw std::runtime_error(error_msg);
    }

    config.azimuth_window = {azimuth_window_start, azimuth_window_end};

    return config;
}

uint8_t OusterSensor::compose_config_flags(
    const sensor::sensor_config& config) {
    uint8_t config_flags = 0;
    if (config.udp_dest) {
        NODELET_INFO_STREAM("Will send UDP data to "
                            << config.udp_dest.value());
        // TODO: revise multicast setup inference
        if (sensor::in_multicast(*config.udp_dest)) {
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
        config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
    }

    if (force_sensor_reinit) {
        force_sensor_reinit = false;
        NODELET_INFO("Forcing sensor to reinitialize");
        config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
    }

    if (persist_config) {
        persist_config =
            false;  // avoid persisting configs implicitly on restarts
        NODELET_INFO("Configuration will be persisted");
        config_flags |= ouster::sensor::CONFIG_PERSIST;
    }

    return config_flags;
}

bool OusterSensor::configure_sensor(const std::string& hostname,
                                    sensor::sensor_config& config) {
    // TODO[UN]: in future always get_config
    if (config.udp_dest && sensor::in_multicast(config.udp_dest.value()) &&
        !mtp_main) {
        if (!get_config(hostname, config, true)) {
            NODELET_ERROR("Error getting active config");
            return false;
        }

        NODELET_INFO("Retrived active config of sensor");
        return true;
    }

    uint8_t config_flags = compose_config_flags(config);
    NODELET_INFO_STREAM("Contacting sensor " << hostname << " ...");
    try {
        set_config(hostname, config, config_flags);
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
    sensor::sensor_info& info, sensor::lidar_mode specified_lidar_mode) {
    if (!info.name.size()) info.name = "UNKNOWN";
    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_from_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        NODELET_WARN(
            "Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        NODELET_WARN("Firmware < %s not supported; output may not be reliable",
                     to_string(sensor::min_version).c_str());

    if (!info.mode) {
        NODELET_WARN(
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        NODELET_ERROR("Beam angles not found in metadata; using design values");
        info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
        info.beam_altitude_angles = sensor::gen1_altitude_angles;
    }
}

void OusterSensor::on_metadata_updated(const sensor::sensor_info&) {}

void OusterSensor::metadata_updated(const sensor::sensor_info& info) {
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
    auto& pf = sensor::get_format(info);
    lidar_packet.buf.resize(pf.lidar_packet_size);
    lidar_packet_msg.buf.resize(pf.lidar_packet_size);
    imu_packet.buf.resize(pf.imu_packet_size);
    imu_packet_msg.buf.resize(pf.imu_packet_size);
}

bool OusterSensor::init_id_changed(const sensor::packet_format& pf,
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

void OusterSensor::read_lidar_packet(sensor::client& cli,
                                     const sensor::packet_format& pf) {
    if (sensor::read_lidar_packet(cli, lidar_packet)) {
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

void OusterSensor::read_imu_packet(sensor::client& cli,
                                   const sensor::packet_format&) {
    if (sensor::read_imu_packet(cli, imu_packet)) {
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

void OusterSensor::connection_loop(sensor::client& cli,
                                   const sensor::packet_format& pf) {
    auto state = sensor::poll_client(cli);
    if (state == sensor::EXIT) {
        NODELET_INFO("poll_client: caught signal, exiting!");
        return;
    }
    if (state & sensor::CLIENT_ERROR || state == sensor::TIMEOUT) {
        handle_poll_client_error();
        return;
    }
    poll_client_error_count = 0;
    if (state & sensor::LIDAR_DATA) {
        read_lidar_packet(cli, pf);
    }
    if (state & sensor::IMU_DATA) {
        read_imu_packet(cli, pf);
    }
}

void OusterSensor::start_sensor_connection_thread() {
    sensor_connection_active = true;
    sensor_connection_thread = std::make_unique<std::thread>([this]() {
        NODELET_DEBUG("sensor_connection_thread active.");
        auto& pf = sensor::get_format(info);
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
