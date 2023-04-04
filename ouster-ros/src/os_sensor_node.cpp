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

#include <fstream>
#include <string>
#include <tuple>
#include <chrono>
#include <vector>

#include <std_srvs/srv/empty.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include "ouster_msgs/msg/packet_msg.hpp"
#include "ouster_srvs/srv/get_config.hpp"
#include "ouster_srvs/srv/set_config.hpp"
#include "ouster_ros/visibility_control.h"
#include "ouster_ros/os_sensor_node_base.h"

namespace sensor = ouster::sensor;
using lifecycle_msgs::srv::ChangeState;
using ouster_msgs::msg::PacketMsg;
using ouster_srvs::srv::GetConfig;
using ouster_srvs::srv::SetConfig;
using rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using sensor::UDPProfileLidar;

using namespace std::chrono_literals;
using namespace std::string_literals;

namespace ouster_ros {

class OusterSensor : public OusterSensorNodeBase {
   public:
    OUSTER_ROS_PUBLIC
    explicit OusterSensor(const rclcpp::NodeOptions& options)
        : OusterSensorNodeBase("os_sensor", options) {
        declare_parameters();
        change_state_client =
            create_client<ChangeState>(get_name() + "/change_state"s);
    }

    LifecycleNodeInterface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State&) {
        RCLCPP_DEBUG(get_logger(), "on_configure() is called.");

        try {
            sensor_hostname = get_sensor_hostname();
            auto config = staged_config.empty()
                              ? parse_config_from_ros_parameters()
                              : parse_config_from_staged_config_string();
            configure_sensor(sensor_hostname, config);
            sensor_client = create_sensor_client(sensor_hostname, config);
            if (!sensor_client)
                return LifecycleNodeInterface::CallbackReturn::FAILURE;
            update_config_and_metadata(*sensor_client);
            save_metadata();
            create_reset_service();
            create_get_metadata_service();
            create_get_config_service();
            create_set_config_service();
            create_publishers();
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

    LifecycleNodeInterface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_activate() is called.");
        LifecycleNode::on_activate(state);
        lidar_packet_pub->on_activate();
        imu_packet_pub->on_activate();
       
        allocate_buffers();
        if (!connection_loop_timer) {
            // TOOD: replace with a thread instead?
            connection_loop_timer =
                rclcpp::create_timer(this, get_clock(), 0s, [this]() {
                    if (reset_in_progress) return;
                    auto& pf = sensor::get_format(info);
                    connection_loop(*sensor_client, pf);
                });
        } else {
            connection_loop_timer->reset();
        }

        reset_in_progress = false;
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_error(
        const rclcpp_lifecycle::State&) {
        RCLCPP_DEBUG(get_logger(), "on_error() is called.");
        // Always return failure for now
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    LifecycleNodeInterface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");
        LifecycleNode::on_deactivate(state);
        connection_loop_timer->cancel();

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State&) {
        RCLCPP_DEBUG(get_logger(), "on_cleanup() is called.");

        try {
            cleanup();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "exception thrown durng cleanup, details: " << ex.what());
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    LifecycleNodeInterface::CallbackReturn on_shutdown(
        const rclcpp_lifecycle::State& state) {
        RCLCPP_DEBUG(get_logger(), "on_shutdown() is called.");

        if (state.label() == "unconfigured") {
            // nothing to do, return success
            return LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        if (state.label() == "active") {
            // stop the timer first then perform cleanup
            connection_loop_timer->cancel();
        }

        // whether state was 'active' or 'inactive' do cleanup
        try {
            cleanup();
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "exception thrown durng cleanup, details: " << ex.what());
            return LifecycleNodeInterface::CallbackReturn::ERROR;
        }

        change_state_client.reset();
        return LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

   private:
    void declare_parameters() {
        declare_parameter("sensor_hostname");
        declare_parameter("metadata");
        declare_parameter("udp_dest");
        declare_parameter("mtp_dest");
        declare_parameter("mtp_main", false);
        declare_parameter("lidar_port", 0);
        declare_parameter("imu_port", 0);
        declare_parameter("lidar_mode");
        declare_parameter("timestamp_mode");
        declare_parameter("udp_profile_lidar");
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
            active_config.clear();
            cached_metadata.clear();
            return false;
        }

        active_config = sensor::to_string(config);

        try {
            cached_metadata = sensor::get_metadata(cli, 60, false);
        } catch (const std::exception& e) {
            RCLCPP_ERROR_STREAM(get_logger(),
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

        return active_config.size() > 0 && cached_metadata.size() > 0;
    }

    void save_metadata() {
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
        if (!write_metadata(meta_file, cached_metadata)) {
            RCLCPP_ERROR(get_logger(),
                         "Exiting because of failure to write metadata path");
            throw std::runtime_error("Failure to write metadata path");
        }
    }

    template <typename CallbackT, typename... CallbackT_Args>
    bool change_state(std::uint8_t transition_id, CallbackT callback,
                      CallbackT_Args... callback_args,
                      std::chrono::seconds time_out = 3s) {
        if (!change_state_client->wait_for_service(time_out)) {
            RCLCPP_ERROR_STREAM(get_logger(),
                                "Service "
                                    << change_state_client->get_service_name()
                                    << "is not available.");
            return false;
        }

        auto request = std::make_shared<ChangeState::Request>();
        request->transition.id = transition_id;
        // send an async request to perform the transition
        change_state_client->async_send_request(
            request, [callback, callback_args...](
                         rclcpp::Client<ChangeState>::SharedFuture) {
                callback(callback_args...);
            });
        return true;
    }

    std::string transition_id_to_string(uint8_t transition_id) {
        switch (transition_id) {
            case lifecycle_msgs::msg::Transition::TRANSITION_CREATE:
                return "create"s;
            case lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE:
                return "configure"s;
            case lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP:
                return "cleanup"s;
            case lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE:
                return "activate";
            case lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE:
                return "deactivate"s;
            case lifecycle_msgs::msg::Transition::TRANSITION_DESTROY:
                return "destroy"s;
            default:
                return "unknown"s;
        }
    }

    void execute_transitions_sequence(std::vector<uint8_t> transitions_sequence,
                                      size_t at) {
        assert(at < transitions_sequence.size() &&
               "at index exceeds the number of transitions");
        auto transition_id = transitions_sequence[at];
        RCLCPP_DEBUG_STREAM(get_logger(),
                            "transition: ["
                                << transition_id_to_string(transition_id)
                                << "] started");
        change_state(transition_id, [this, transitions_sequence, at]() {
            RCLCPP_DEBUG_STREAM(get_logger(),
                                "transition: [" << transition_id_to_string(
                                                       transitions_sequence[at])
                                                << "] completed");
            if (at < transitions_sequence.size() - 1) {
                execute_transitions_sequence(transitions_sequence, at + 1);
            } else {
                RCLCPP_DEBUG_STREAM(get_logger(),
                                    "transitions sequence completed");
            }
        });
    }

    // param init_id_reset is overriden to true when force_reinit is true
    void reset_sensor(bool force_reinit, bool init_id_reset = false) {
        if (reset_in_progress) {
            RCLCPP_WARN(
                get_logger(),
                "sensor reset is already in progress, ignoring second call!");
            return;
        }

        reset_in_progress = true;
        force_sensor_reinit = force_reinit;
        connection_loop_timer->cancel();
        reset_last_init_id = force_reinit ? true : init_id_reset;
        auto request_transitions = std::vector<uint8_t>{
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
            lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP,
            lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE,
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
        execute_transitions_sequence(request_transitions, 0);
    }

    // TODO: need to notify dependent node(s) of the update
    void reactivate_sensor(bool init_id_reset = false) {
        if (reset_in_progress) {
            RCLCPP_WARN(
                get_logger(),
                "sensor reset is already in progress, ignoring second call!");
            return;
        }

        reset_in_progress = true;
        connection_loop_timer->cancel();
        reset_last_init_id = init_id_reset;
        update_config_and_metadata(*sensor_client);
        save_metadata();
        auto request_transitions = std::vector<uint8_t>{
            lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE,
            lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE};
        execute_transitions_sequence(request_transitions, 0);
    }

    void create_reset_service() {
        reset_srv = create_service<std_srvs::srv::Empty>(
            "reset",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
                   std::shared_ptr<std_srvs::srv::Empty::Response>) {
                RCLCPP_INFO(get_logger(), "reset service invoked");
                reset_sensor(true);
            });

        RCLCPP_INFO(get_logger(), "reset service created");
    }

    void create_get_config_service() {
        get_config_srv = create_service<GetConfig>(
            "get_config",
            [this](const std::shared_ptr<GetConfig::Request>,
                   std::shared_ptr<GetConfig::Response> response) {
                response->config = active_config;
                return active_config.size() > 0;
            });

        RCLCPP_INFO(get_logger(), "get_config service created");
    }

    void create_set_config_service() {
        set_config_srv = create_service<SetConfig>(
            "set_config",
            [this](const std::shared_ptr<SetConfig::Request> request,
                   std::shared_ptr<SetConfig::Response> response) {
                response->config = "";

                try {
                    staged_config = load_config_file(request->config_file);
                    if (staged_config.empty()) {
                        RCLCPP_ERROR_STREAM(
                            get_logger(),
                            "provided config file: "
                                << request->config_file
                                << " turned to be empty. set_config ignored!");
                        return false;
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR_STREAM(
                        get_logger(),
                        "exception thrown while loading config file: "
                            << request->config_file
                            << ", exception details: " << e.what());
                    return false;
                }

                response->config = staged_config;
                // TODO: this is currently set to force_reinit but it doesn't
                // need to be the case if it was possible to know that the new
                // config would result in a reinit when a reinit is not forced
                reset_sensor(true);
                return true;
            });

        RCLCPP_INFO(get_logger(), "set_config service created");
    }

    std::shared_ptr<sensor::client> create_sensor_client(
        const std::string& hostname, const sensor::sensor_config& config) {
        RCLCPP_INFO_STREAM(get_logger(), "Starting sensor "
                                             << hostname
                                             << " initialization...");

        int lidar_port =
            config.udp_port_lidar ? config.udp_port_lidar.value() : 0;
        int imu_port = config.udp_port_imu ? config.udp_port_imu.value() : 0;
        auto udp_dest = config.udp_dest ? config.udp_dest.value() : "";

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

    sensor::sensor_config parse_config_from_ros_parameters() {
        auto udp_dest = get_parameter("udp_dest").as_string();
        auto mtp_dest_arg = get_parameter("mtp_dest").as_string();
        auto mtp_main_arg = get_parameter("mtp_main").as_bool();
        auto lidar_port = get_parameter("lidar_port").as_int();
        auto imu_port = get_parameter("imu_port").as_int();
        auto lidar_mode_arg = get_parameter("lidar_mode").as_string();
        auto timestamp_mode_arg = get_parameter("timestamp_mode").as_string();
        auto udp_profile_lidar_arg =
            get_parameter("udp_profile_lidar").as_string();

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

        nonstd::optional<sensor::UDPProfileLidar> udp_profile_lidar;
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

        config.udp_profile_lidar = udp_profile_lidar;
        config.operating_mode = sensor::OPERATING_NORMAL;
        if (lidar_mode) config.ld_mode = lidar_mode;
        if (timestamp_mode) config.ts_mode = timestamp_mode;
        if (is_arg_set(udp_dest)) {
            config.udp_dest = udp_dest;
            if (sensor::in_multicast(udp_dest)) {
                mtp_dest =
                    is_arg_set(mtp_dest_arg) ? mtp_dest_arg : std::string{};
                mtp_main = mtp_main_arg;
            }
        }

        return config;
    }

    sensor::sensor_config parse_config_from_staged_config_string() {
        sensor::sensor_config config = sensor::parse_config(staged_config);
        staged_config.clear();
        return config;
    }

    uint8_t compose_config_flags(const sensor::sensor_config& config) {
        uint8_t config_flags = 0;
        if (config.udp_dest) {
            RCLCPP_INFO_STREAM(get_logger(), "Will send UDP data to "
                                                 << config.udp_dest.value());
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

        return config_flags;
    }

    void configure_sensor(const std::string& hostname,
                          sensor::sensor_config& config) {
        if (config.udp_dest && sensor::in_multicast(config.udp_dest.value()) &&
            !mtp_main) {
            if (!get_config(hostname, config, true)) {
                RCLCPP_ERROR(get_logger(), "Error getting active config");
            } else {
                RCLCPP_INFO(get_logger(), "Retrived active config of sensor");
            }
            return;
        }

        uint8_t config_flags = compose_config_flags(config);
        if (!set_config(hostname, config, config_flags)) {
            throw std::runtime_error("Error connecting to sensor " + hostname);
        }

        RCLCPP_INFO_STREAM(get_logger(),
                           "Sensor " << hostname << " configured successfully");
    }

    std::string load_config_file(const std::string& config_file) {
        std::ifstream ifs{};
        ifs.open(config_file);
        if (ifs.fail()) return {};
        std::stringstream buf;
        buf << ifs.rdbuf();
        return buf.str();
    }

    // fill in values that could not be parsed from metadata
    void populate_metadata_defaults(sensor::sensor_info& info,
                                    sensor::lidar_mode specified_lidar_mode) {
        if (!info.name.size()) info.name = "UNKNOWN";

        if (!info.sn.size()) info.sn = "UNKNOWN";

        ouster::util::version v = ouster::util::version_of_string(info.fw_rev);
        if (v == ouster::util::invalid_version)
            RCLCPP_WARN(
                get_logger(),
                "Unknown sensor firmware version; output may not be reliable");
        else if (v < sensor::min_version)
            RCLCPP_WARN(
                get_logger(),
                "Firmware < %s not supported; output may not be reliable",
                to_string(sensor::min_version).c_str());

        if (!info.mode) {
            RCLCPP_WARN(
                get_logger(),
                "Lidar mode not found in metadata; output may not be reliable");
            info.mode = specified_lidar_mode;
        }

        if (!info.prod_line.size()) info.prod_line = "UNKNOWN";

        if (info.beam_azimuth_angles.empty() ||
            info.beam_altitude_angles.empty()) {
            RCLCPP_ERROR(
                get_logger(),
                "Beam angles not found in metadata; using design values");
            info.beam_azimuth_angles = sensor::gen1_azimuth_angles;
            info.beam_altitude_angles = sensor::gen1_altitude_angles;
        }
    }

    // try to write metadata file
    bool write_metadata(const std::string& meta_file,
                        const std::string& metadata) {
        std::ofstream ofs(meta_file);
        if (ofs.is_open()) {
            ofs << metadata << std::endl;
            ofs.close();
            RCLCPP_INFO_STREAM(get_logger(),
                               "Wrote sensor metadata to " << meta_file);
        } else {
            RCLCPP_WARN(
                get_logger(),
                "Failed to write metadata to %s; check that the path is valid. "
                "If you provided a relative path, please note that the working "
                "directory of all ROS nodes is set by default to $ROS_HOME",
                meta_file.c_str());
            return false;
        }
        return true;
    }

    void create_publishers() {
        rclcpp::SensorDataQoS qos;
        lidar_packet_pub = create_publisher<PacketMsg>("lidar_packets", qos);
        imu_packet_pub = create_publisher<PacketMsg>("imu_packets", qos);
    }

    void allocate_buffers() {
        auto& pf = sensor::get_format(info);
        lidar_packet.buf.resize(pf.lidar_packet_size + 1);
        imu_packet.buf.resize(pf.imu_packet_size + 1);
    }

    bool init_id_changed(const sensor::packet_format& pf,
                         const uint8_t* lidar_buf) {
        uint32_t current_init_id = pf.init_id(lidar_buf);
        static uint32_t last_init_id = current_init_id + 1;
        if (reset_last_init_id && last_init_id != current_init_id) {
            last_init_id = current_init_id;
            reset_last_init_id = false;
            return false;
        }
        if (last_init_id == current_init_id) return false;
        last_init_id = current_init_id;
        return true;
    }

    static bool is_non_legacy_lidar_profile(const sensor::sensor_info& info) {
        return info.format.udp_profile_lidar !=
               UDPProfileLidar::PROFILE_LIDAR_LEGACY;
    }

    void handle_poll_client_error() {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 100,
                             "sensor::poll_client()) returned error");
        // in case error continues for a while attempt to recover by
        // performing sensor reset
        if (++poll_client_error_count > max_poll_client_error_count) {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "maximum number of allowed errors from "
                "sensor::poll_client() reached, performing self reset...");
            poll_client_error_count = 0;
            reset_sensor(true);
        }
    }

    void handle_lidar_packet(sensor::client& cli,
                             const sensor::packet_format& pf) {
        if (sensor::read_lidar_packet(cli, lidar_packet.buf.data(), pf)) {
            read_lidar_packet_errors = 0;
            if (is_non_legacy_lidar_profile(info) &&
                init_id_changed(pf, lidar_packet.buf.data())) {
                // TODO: short circut reset if no breaking changes occured?
                RCLCPP_WARN(get_logger(),
                            "sensor init_id has changed! reactivating..");
                reactivate_sensor();
            } else {
                lidar_packet_pub->publish(lidar_packet);
            }
        } else {
            if (++read_lidar_packet_errors > max_read_lidar_packet_errors) {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "maximum number of allowed errors from "
                    "sensor::read_lidar_packet() reached, reactivating...");
                read_lidar_packet_errors = 0;
                reactivate_sensor(true);
            }
        }
    }

    void handle_imu_packet(sensor::client& cli,
                           const sensor::packet_format& pf) {
        if (sensor::read_imu_packet(cli, imu_packet.buf.data(), pf)) {
            imu_packet_pub->publish(imu_packet);
        } else {
            if (++read_imu_packet_errors > max_read_imu_packet_errors) {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "maximum number of allowed errors from "
                    "sensor::read_imu_packet() reached, reactivating...");
                read_imu_packet_errors = 0;
                reactivate_sensor(true);
            }
        }
    }

    void connection_loop(sensor::client& cli, const sensor::packet_format& pf) {
        auto state = sensor::poll_client(cli);
        if (state == sensor::EXIT) {
            RCLCPP_INFO(get_logger(), "poll_client: caught signal, exiting!");
            return;
        }
        if (state & sensor::CLIENT_ERROR) {
            handle_poll_client_error();
            return;
        }
        poll_client_error_count = 0;
        if (state & sensor::LIDAR_DATA) {
            handle_lidar_packet(cli, pf);
        }
        if (state & sensor::IMU_DATA) {
            handle_imu_packet(cli, pf);
        }
    }

    void cleanup() {
        sensor_client.reset();
        lidar_packet_pub.reset();
        imu_packet_pub.reset();
        get_metadata_srv.reset();
        get_config_srv.reset();
        set_config_srv.reset();
        connection_loop_timer.reset();
    }

   private:
    std::string sensor_hostname;
    std::string staged_config;
    std::string active_config;
    std::string mtp_dest;
    bool mtp_main;
    std::shared_ptr<sensor::client> sensor_client;
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    rclcpp_lifecycle::LifecyclePublisher<PacketMsg>::SharedPtr lidar_packet_pub;
    rclcpp_lifecycle::LifecyclePublisher<PacketMsg>::SharedPtr imu_packet_pub;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
    rclcpp::Service<GetConfig>::SharedPtr get_config_srv;
    rclcpp::Service<SetConfig>::SharedPtr set_config_srv;
    std::shared_ptr<rclcpp::Client<ChangeState>> change_state_client;
    std::shared_ptr<rclcpp::TimerBase> connection_loop_timer;

    bool force_sensor_reinit = false;
    bool reset_last_init_id = true;
    std::atomic<bool> reset_in_progress = {false};

    // TODO: add as a ros parameter
    const int max_poll_client_error_count = 10;
    int poll_client_error_count = 0;
    // TODO: add as a ros parameter
    const int max_read_lidar_packet_errors = 60;
    int read_lidar_packet_errors = 0;
    // TODO: add as a ros parameter
    const int max_read_imu_packet_errors = 60;
    int read_imu_packet_errors = 0;
};

}  // namespace ouster_ros

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(ouster_ros::OusterSensor)
