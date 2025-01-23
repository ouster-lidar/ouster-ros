/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_sensor_node.cpp
 * @brief A node that connects to a live ouster sensor
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <chrono>

#include "os_sensor_node.h"

using ouster_sensor_msgs::msg::PacketMsg;
using ouster_sensor_msgs::srv::GetConfig;
using ouster_sensor_msgs::srv::SetConfig;

using std::to_string;
using namespace std::chrono_literals;

using sensor::ImuPacket;
using sensor::LidarPacket;

namespace ouster_ros {

OusterSensor::OusterSensor(const std::string& name,
                           const rclcpp::NodeOptions& options)
    : OusterSensorNodeBase(name, options) {
    declare_parameters();
    staged_config = parse_config_from_ros_parameters();
    attempt_reconnect = get_parameter("attempt_reconnect").as_bool();
    dormant_period_between_reconnects = 
        get_parameter("dormant_period_between_reconnects").as_double();
    reconnect_attempts_available =
        get_parameter("max_failed_reconnect_attempts").as_int();

    bool auto_start = get_parameter("auto_start").as_bool();

    if (auto_start) {
        RCLCPP_INFO(get_logger(), "auto start requested");
        reconnect_timer = create_wall_timer(1s, [this]() {
            reconnect_timer->cancel();
            auto request_transitions = std::vector<uint8_t>{
                lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
            execute_transitions_sequence(request_transitions, 0);
            RCLCPP_INFO(get_logger(), "auto start initiated");
        });
    }
}

OusterSensor::OusterSensor(const rclcpp::NodeOptions& options)
    : OusterSensor("os_sensor", options) {}

OusterSensor::~OusterSensor() {
    RCLCPP_DEBUG(get_logger(), "OusterSensor::~OusterSensor() called");
    stop_sensor_connection_thread();
}

void OusterSensor::declare_parameters() {
    declare_parameter("sensor_hostname", "");
    declare_parameter("lidar_ip", "");      // community driver param
    declare_parameter("metadata", "");
    declare_parameter("udp_dest", "");
    declare_parameter("computer_ip", "");   // community driver param
    declare_parameter("mtp_dest", "");
    declare_parameter("mtp_main", false);
    declare_parameter("lidar_port", 0);
    declare_parameter("imu_port", 0);
    declare_parameter("lidar_mode", "");
    declare_parameter("timestamp_mode", "");
    declare_parameter("udp_profile_lidar", "");
    declare_parameter("use_system_default_qos", false);
    declare_parameter("azimuth_window_start", MIN_AZW);
    declare_parameter("azimuth_window_end", MAX_AZW);
    declare_parameter("persist_config", false);
    declare_parameter("attempt_reconnect", false);
    declare_parameter("dormant_period_between_reconnects", 1.0);
    declare_parameter("max_failed_reconnect_attempts", INT_MAX);
    declare_parameter("auto_start", false);
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
        if (!get_active_config_no_throw(sensor_hostname, config))
            return false;

        RCLCPP_INFO(get_logger(), "Retrived sensor active config");
        // Unfortunately it seems we need to invoke this to force the auto
        // TODO[UN]: find a shortcut
        // Only reset udp_dest if auto_udp was allowed on startup
        if (auto_udp_allowed) config.udp_dest.reset();
        if (!configure_sensor(sensor_hostname, config))
            return false;
    }

    reset_last_init_id = true;
    sensor_client = create_sensor_client(sensor_hostname, config);
    if (!sensor_client)
        return false;

    create_metadata_pub();
    update_metadata(*sensor_client);
    create_services();

    return true;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_configure(
    const rclcpp_lifecycle::State&) {
    RCLCPP_DEBUG(get_logger(), "on_configure() is called.");

    try {
        if (!start()) {
            auto sleep_duration = std::chrono::duration<double>(dormant_period_between_reconnects);
            reconnect_timer = create_wall_timer(sleep_duration, [this]() {
                reconnect_timer->cancel();
                if (attempt_reconnect && reconnect_attempts_available-- > 0) {
                    RCLCPP_INFO_STREAM(get_logger(), "Attempting to communicate with the sensor, "
                                        "remaining attempts: " << reconnect_attempts_available);

                    auto request_transitions = std::vector<uint8_t>{
                        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
                        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
                    execute_transitions_sequence(request_transitions, 0);
                }
            });
            return LifecycleNodeInterface::CallbackReturn::FAILURE;
        } else {
            // reset counter
            reconnect_attempts_available =
                get_parameter("max_failed_reconnect_attempts").as_int();
        }
    } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "exception thrown while configuring the sensor, details: "
                << ex.what());
        // TODO: return ERROR on fatal errors, FAILURE otherwise
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_activate(
    const rclcpp_lifecycle::State& state) {
    RCLCPP_DEBUG(get_logger(), "on_activate() is called.");
    LifecycleNode::on_activate(state);
    if (cached_metadata.empty())
        update_metadata(*sensor_client);
    create_publishers();
    allocate_buffers();
    start_sensor_connection_thread();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_error(
    const rclcpp_lifecycle::State&) {
    RCLCPP_DEBUG(get_logger(), "on_error() is called.");
    // Always return failure for now
    return LifecycleNodeInterface::CallbackReturn::FAILURE;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_deactivate(
    const rclcpp_lifecycle::State& state) {
    RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");
    LifecycleNode::on_deactivate(state);
    stop_sensor_connection_thread();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_cleanup(
    const rclcpp_lifecycle::State&) {
    RCLCPP_DEBUG(get_logger(), "on_cleanup() is called.");

    try {
        cleanup();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "exception thrown during cleanup, details: " << ex.what());
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

LifecycleNodeInterface::CallbackReturn OusterSensor::on_shutdown(
    const rclcpp_lifecycle::State& state) {
    RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");

    if (state.label() == "unconfigured") {
        // nothing to do, return success
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    if (state.label() == "active") {
        stop_sensor_connection_thread();
    }

    // whether state was 'active' or 'inactive' do cleanup
    try {
        cleanup();
    } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "exception thrown during cleanup, details: " << ex.what());
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    change_state_client.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

std::string OusterSensor::get_sensor_hostname() {
    auto hostname = get_parameter("sensor_hostname").as_string();
    if (!is_arg_set(hostname)) {
        hostname = get_parameter("lidar_ip").as_string();
        if (!is_arg_set(hostname)) {
            auto error_msg = "Must specify a sensor hostname";
            RCLCPP_FATAL_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    return hostname;
}

void OusterSensor::update_metadata(sensor::client& cli) {
    try {
        cached_metadata = sensor::get_metadata(cli, 60, false);
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(get_logger(),
                            "sensor::get_metadata exception: " << e.what());
        cached_metadata.clear();
    }

    if (cached_metadata.empty()) {
        const auto error_msg = "Failed to collect sensor metadata";
        RCLCPP_ERROR(get_logger(), error_msg);
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
    auto meta_file = get_parameter("metadata").as_string();
    if (!is_arg_set(meta_file)) {
        meta_file = sensor_hostname.substr(0, sensor_hostname.rfind('.')) +
                    "-metadata.json";
        RCLCPP_INFO_STREAM(
            get_logger(),
            "No metadata file was specified, using: " << meta_file);
    }

    // write metadata file. If metadata_path is relative, will use cwd
    // (usually ~/.ros)
    if (write_text_to_file(meta_file, cached_metadata)) {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Wrote sensor metadata to " << meta_file);
    } else {
        RCLCPP_WARN_STREAM(
            get_logger(),
            "Failed to write metadata to "
                << meta_file
                << "; check that the path is valid. If you provided a relative "
                   "path, please note that the working directory of all ROS "
                   "nodes is set by default to $ROS_HOME");
    }
}

// param init_id_reset is overriden to true when force_reinit is true
void OusterSensor::reset_sensor(bool force_reinit, bool init_id_reset) {
    if (!sensor_connection_active) {
        RCLCPP_WARN(get_logger(),
                    "sensor reset is invoked but sensor connection is not "
                    "active, ignoring call!");
        return;
    }

    force_sensor_reinit = force_reinit;
    reset_last_init_id = force_reinit ? true : init_id_reset;
    auto request_transitions = std::vector<uint8_t>{
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
        lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
        lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
    execute_transitions_sequence(request_transitions, 0);
}

// TODO: need to notify dependent node(s) of the update
void OusterSensor::reactivate_sensor(bool init_id_reset) {
    if (!sensor_connection_active) {
        // This may indicate that we are in the process of re-activation
        RCLCPP_WARN(get_logger(),
                    "sensor reactivate is invoked but sensor connection is "
                    "not active, ignoring call!");
        return;
    }

    reset_last_init_id = init_id_reset;
    cached_metadata.clear();
    auto request_transitions = std::vector<uint8_t>{
        lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
        lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
    execute_transitions_sequence(request_transitions, 0);
}

void OusterSensor::create_reset_service() {
    reset_srv = create_service<std_srvs::srv::Empty>(
        "reset", [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                        std::shared_ptr<std_srvs::srv::Empty::Response>) {
            RCLCPP_INFO(get_logger(), "reset service invoked");
            reset_sensor(true);
        });

    RCLCPP_INFO(get_logger(), "reset service created");
}

bool OusterSensor::get_active_config_no_throw(
    const std::string& sensor_hostname, sensor::sensor_config& config) {
    try {
        if (get_config(sensor_hostname, config, true))
            return true;
    } catch(const std::exception&) {
        RCLCPP_ERROR_STREAM(
            get_logger(),
            "Couldn't get active config for: " << sensor_hostname);
        return false;
    }

    RCLCPP_ERROR_STREAM(
        get_logger(),
        "Couldn't get active config for: " << sensor_hostname);
    return false;
}

void OusterSensor::create_get_config_service() {
    get_config_srv = create_service<GetConfig>(
        "get_config", [this](const std::shared_ptr<GetConfig::Request>,
                             std::shared_ptr<GetConfig::Response> response) {
            std::string active_config;
            sensor::sensor_config config;
            if (get_active_config_no_throw(sensor_hostname, config))
                active_config = to_string(config);
            response->config = active_config;
            return active_config.size() > 0;
        });

    RCLCPP_INFO(get_logger(), "get_config service created");
}

void OusterSensor::create_set_config_service() {
    set_config_srv = create_service<SetConfig>(
        "set_config", [this](const std::shared_ptr<SetConfig::Request> request,
                             std::shared_ptr<SetConfig::Response> response) {
            response->config = "";
            std::string config_str;
            try {
                config_str = read_text_file(request->config_file);
                if (config_str.empty()) {
                    RCLCPP_ERROR_STREAM(
                        get_logger(),
                        "provided config file: "
                            << request->config_file
                            << " turned to be empty. set_config ignored!");
                    return false;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR_STREAM(
                    get_logger(), "exception thrown while loading config file: "
                                      << request->config_file
                                      << ", exception details: " << e.what());
                return false;
            }

            staged_config = sensor::parse_config(config_str);
            response->config = config_str;
            // TODO: this is currently set to force_reinit but it doesn't
            // need to be the case if it was possible to know that the new
            // config would result in a reinit when a reinit is not forced
            reset_sensor(true);
            return true;
        });

    RCLCPP_INFO(get_logger(), "set_config service created");
}

std::shared_ptr<sensor::client> OusterSensor::create_sensor_client(
    const std::string& hostname, const sensor::sensor_config& config) {

    int lidar_port = config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
    int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;
    auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";

    RCLCPP_INFO_STREAM(get_logger(),
                       "Starting sensor " << hostname << " initialization..."
                       " Using ports: " << lidar_port << "/" << imu_port);

    std::shared_ptr<sensor::client> cli;
    if (sensor::in_multicast(udp_dest)) {
        // use the mtp_init_client to receive data via multicast
        // if mtp_main is true when sensor will be configured
        cli = sensor::mtp_init_client(hostname, config, mtp_dest, mtp_main);
    } else if (lidar_port != 0 && imu_port != 0) {
        // use no-config version of init_client to bind to pre-configured
        // ports
        cli = sensor::init_client(hostname, lidar_port, imu_port);
    } else {
        // use the full init_client to generate and assign random ports to
        // sensor
        cli = sensor::init_client(hostname, udp_dest, sensor::MODE_UNSPEC,
                                  sensor::TIME_FROM_UNSPEC, lidar_port, imu_port);
    }

    if (!cli) {
        auto error_msg = "Failed to initialize client";
        RCLCPP_ERROR_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }

    return cli;
}

sensor::sensor_config OusterSensor::parse_config_from_ros_parameters() {
    auto udp_dest = get_parameter("udp_dest").as_string();
    if (!is_arg_set(udp_dest))
        udp_dest = get_parameter("computer_ip").as_string();

    auto mtp_dest_arg = get_parameter("mtp_dest").as_string();
    auto mtp_main_arg = get_parameter("mtp_main").as_bool();
    auto lidar_port = get_parameter("lidar_port").as_int();
    auto imu_port = get_parameter("imu_port").as_int();
    auto lidar_mode_arg = get_parameter("lidar_mode").as_string();
    auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
    auto udp_profile_lidar_arg = get_parameter("udp_profile_lidar").as_string();
    auto azimuth_window_start = get_parameter("azimuth_window_start").as_int();
    auto azimuth_window_end = get_parameter("azimuth_window_end").as_int();

    if (lidar_port < 0 || lidar_port > 65535) {
        auto error_msg =
            "Invalid lidar port number! port value should be in the range "
            "[0, 65535].";
        RCLCPP_FATAL_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }

    if (imu_port < 0 || imu_port > 65535) {
        auto error_msg =
            "Invalid imu port number! port value should be in the range "
            "[0, 65535].";
        RCLCPP_FATAL_STREAM(get_logger(), error_msg);
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
            RCLCPP_FATAL_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    // set lidar mode from param
    sensor::lidar_mode lidar_mode = sensor::MODE_UNSPEC;
    if (is_arg_set(lidar_mode_arg)) {
        lidar_mode = sensor::lidar_mode_of_string(lidar_mode_arg);
        if (!lidar_mode) {
            auto error_msg = "Invalid lidar mode: " + lidar_mode_arg;
            RCLCPP_FATAL_STREAM(get_logger(), error_msg);
            throw std::runtime_error(error_msg);
        }
    }

    // set timestamp mode from param
    sensor::timestamp_mode timestamp_mode = sensor::TIME_FROM_UNSPEC;
    if (is_arg_set(timestamp_mode_arg)) {
        // In case the option TIME_FROM_ROS_TIME is set then leave the
        // sensor timestamp_mode unmodified
        if (timestamp_mode_arg == "TIME_FROM_ROS_TIME" ||
            timestamp_mode_arg == "TIME_FROM_ROS_RECEPTION") {
            RCLCPP_INFO(get_logger(),
                        "TIME_FROM_ROS_TIME timestamp mode specified."
                        " IMU and pointcloud messages will use ros time");
        } else {
            timestamp_mode =
                sensor::timestamp_mode_of_string(timestamp_mode_arg);
            if (!timestamp_mode) {
                auto error_msg =
                    "Invalid timestamp mode: " + timestamp_mode_arg;
                RCLCPP_FATAL_STREAM(get_logger(), error_msg);
                throw std::runtime_error(error_msg);
            }
        }
    }

    sensor::sensor_config config;
    if (lidar_port == 0) {
        RCLCPP_WARN_EXPRESSION(
            get_logger(), !is_arg_set(mtp_dest_arg),
            "lidar port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_lidar = lidar_port;
    }

    if (imu_port == 0) {
        RCLCPP_WARN_EXPRESSION(
            get_logger(), !is_arg_set(mtp_dest_arg),
            "imu port set to zero, the client will assign a random port "
            "number!");
    } else {
        config.udp_port_imu = imu_port;
    }

    persist_config = get_parameter("persist_config").as_bool();
    if (persist_config && (lidar_port == 0 || imu_port == 0)) {
        RCLCPP_WARN(get_logger(), "When using persist_config it is recommended "
        " to not use 0 for port values as this currently will trigger sensor "
        " reinit event each time");
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
        RCLCPP_FATAL_STREAM(get_logger(), error_msg);
        throw std::runtime_error(error_msg);
    }

    config.azimuth_window = {azimuth_window_start, azimuth_window_end};

    return config;
}

uint8_t OusterSensor::compose_config_flags(
    const sensor::sensor_config& config) {
    uint8_t config_flags = 0;
    if (config.udp_dest) {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Will send UDP data to " << config.udp_dest.value());
        // TODO: revise multicast setup inference
        if (sensor::in_multicast(*config.udp_dest)) {
            if (is_arg_set(mtp_dest)) {
                RCLCPP_INFO_STREAM(
                    get_logger(),
                    "Will recieve data via multicast on " << mtp_dest);
            } else {
                RCLCPP_INFO(
                    get_logger(),
                    "mtp_dest was not set, will recieve data via multicast "
                    "on first available interface");
            }
        }
    } else {
        RCLCPP_INFO(get_logger(), "Will use automatic UDP destination");
        config_flags |= ouster::sensor::CONFIG_UDP_DEST_AUTO;
    }

    if (force_sensor_reinit) {
        force_sensor_reinit = false;
        RCLCPP_INFO(get_logger(), "Forcing sensor to reinitialize");
        config_flags |= ouster::sensor::CONFIG_FORCE_REINIT;
    }

    if (persist_config) {
        persist_config = false; // avoid persisting configs implicitly on restarts
        RCLCPP_INFO(get_logger(), "Configuration will be persisted");
        config_flags |= ouster::sensor::CONFIG_PERSIST;
    }

    return config_flags;
}

bool OusterSensor::configure_sensor(const std::string& hostname,
                                    sensor::sensor_config& config) {
    if (config.udp_dest && sensor::in_multicast(config.udp_dest.value()) &&
        !mtp_main) {
        if (!get_config(hostname, config, true)) {
            RCLCPP_ERROR(get_logger(), "Error getting active config");
            return false;
        }

        RCLCPP_INFO(get_logger(), "Retrived active config of sensor");
        return true;
    }

    uint8_t config_flags = compose_config_flags(config);
    RCLCPP_INFO_STREAM(get_logger(), "Contacting sensor " << hostname << " ...");
    try {
        set_config(hostname, config, config_flags);
    } catch (const std::exception& ex) {
        RCLCPP_ERROR_STREAM(get_logger(), "Error connecting to sensor " << hostname <<
        ", details: " << ex.what());
        return false;
    }

    RCLCPP_INFO_STREAM(get_logger(),
                       "Sensor " << hostname << " configured successfully");
    return true;
}

// fill in values that could not be parsed from metadata
void OusterSensor::populate_metadata_defaults(
    sensor::sensor_info& info, sensor::lidar_mode specified_lidar_mode) {
    if (!info.name.size()) info.name = "UNKNOWN";
    if (!info.sn.size()) info.sn = "UNKNOWN";

    ouster::util::version v = ouster::util::version_from_string(info.fw_rev);
    if (v == ouster::util::invalid_version)
        RCLCPP_WARN(
            get_logger(),
            "Unknown sensor firmware version; output may not be reliable");
    else if (v < sensor::min_version)
        RCLCPP_WARN(get_logger(),
                    "Firmware < %s not supported; output may not be reliable",
                    to_string(sensor::min_version).c_str());

    if (!info.mode) {
        RCLCPP_WARN(
            get_logger(),
            "Lidar mode not found in metadata; output may not be reliable");
        info.mode = specified_lidar_mode;
    }

    if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

    if (info.beam_azimuth_angles.empty() || info.beam_altitude_angles.empty()) {
        RCLCPP_ERROR(get_logger(),
                     "Beam angles not found in metadata; using design values");
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
    bool use_system_default_qos =
        get_parameter("use_system_default_qos").as_bool();
    rclcpp::QoS system_default_qos = rclcpp::SystemDefaultsQoS();
    rclcpp::QoS sensor_data_qos = rclcpp::SensorDataQoS();
    auto selected_qos =
        use_system_default_qos ? system_default_qos : sensor_data_qos;
    lidar_packet_pub =
        create_publisher<PacketMsg>("lidar_packets", selected_qos);
    imu_packet_pub = create_publisher<PacketMsg>("imu_packets", selected_qos);
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
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100,
                         "sensor::poll_client()) returned error or timed out");
    // in case error continues for a while attempt to recover by
    // performing sensor reset
    if (++poll_client_error_count > max_poll_client_error_count) {
        RCLCPP_ERROR(
            get_logger(),
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
        if (!is_legacy_lidar_profile(info) && init_id_changed(pf, lidar_packet)) {
            // TODO: short circut reset if no breaking changes occured?
            RCLCPP_WARN(get_logger(), "sensor init_id has changed! reactivating..");
            reset_sensor(false);
        }
        handle_lidar_packet(lidar_packet);
    } else {
        if (++read_lidar_packet_errors > max_read_lidar_packet_errors) {
            RCLCPP_ERROR(
                get_logger(),
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
        on_imu_packet_msg(imu_packet);
    } else {
        if (++read_imu_packet_errors > max_read_imu_packet_errors) {
            RCLCPP_ERROR(
                get_logger(),
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

void OusterSensor::cleanup() {
    sensor_client.reset();
    lidar_packet_pub.reset();
    imu_packet_pub.reset();
    get_metadata_srv.reset();
    get_config_srv.reset();
    set_config_srv.reset();
    sensor_connection_thread.reset();
}

void OusterSensor::connection_loop(sensor::client& cli,
                                   const sensor::packet_format& pf) {
    auto state = sensor::poll_client(cli);
    if (state == sensor::EXIT) {
        RCLCPP_INFO(get_logger(), "poll_client: caught signal, exiting!");
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
        RCLCPP_DEBUG(get_logger(), "sensor_connection_thread active.");
        auto& pf = sensor::get_format(info);
        while (rclcpp::ok() && sensor_connection_active) {
            connection_loop(*sensor_client, pf);
        }
        RCLCPP_DEBUG(get_logger(), "sensor_connection_thread done.");
    });
}

void OusterSensor::stop_sensor_connection_thread() {
    RCLCPP_DEBUG(get_logger(), "sensor_connection_thread stopping.");
    if (sensor_connection_thread != nullptr &&
        sensor_connection_thread->joinable()) {
        sensor_connection_active = false;
        sensor_connection_thread->join();
    }
}

void OusterSensor::on_lidar_packet_msg(const LidarPacket&) {
    lidar_packet_msg.buf.swap(lidar_packet.buf);
    lidar_packet_pub->publish(lidar_packet_msg);
}

void OusterSensor::on_imu_packet_msg(const ImuPacket&) {
    imu_packet_msg.buf.swap(imu_packet.buf);
    imu_packet_pub->publish(imu_packet_msg);
}

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterSensor)
