/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file os_pcap_nodelet.cpp
 * @brief This nodelet mainly handles publishing saved metadata
 *
 */

// prevent clang-format from altering the location of "ouster_ros/ros.h", the
// header file needs to be the first include due to PCL_NO_PRECOMPILE flag
// clang-format off
#include "ouster_ros/os_ros.h"
// clang-format on

#include <pluginlib/class_list_macros.h>

#include <string>
#include <thread>

#include "ouster_ros/os_sensor_nodelet_base.h"
#include "ouster_ros/PacketMsg.h"
#include "thread_safe_ring_buffer.h"
#include <ouster/os_pcap.h>

namespace sensor = ouster::sensor;

namespace ouster_ros {

class OusterPcap : public OusterSensorNodeletBase {
   private:
    virtual void onInit() override {
        auto meta_file = get_meta_file();
        auto pcap_file = get_pcap_file();
        create_metadata_publisher();
        load_metadata_from_file(meta_file);
        allocate_buffers();
        create_publishers();
        open_pcap(pcap_file);
        publish_metadata();
        create_get_metadata_service();
        start_packet_processing_threads();
        start_packet_read_thread();
        NODELET_INFO("Running in replay mode");
    }

    std::string get_meta_file() const {
        auto meta_file =
            getPrivateNodeHandle().param("metadata", std::string{});
        if (!is_arg_set(meta_file)) {
            NODELET_ERROR("Must specify metadata file in replay mode");
            throw std::runtime_error("metadata no specificed");
        }
        return meta_file;
    }

    std::string get_pcap_file() const {
        auto pcap_file =
            getPrivateNodeHandle().param("pcap_file", std::string{});
        if (!is_arg_set(pcap_file)) {
            NODELET_ERROR("Must specify pcap file in pcap replay mode");
            throw std::runtime_error("metadata no specificed");
        }
        return pcap_file;
    }

    void load_metadata_from_file(const std::string& meta_file) {
        try {
            cached_metadata = read_text_file(meta_file);
            info = sensor::parse_metadata(cached_metadata);
            display_lidar_info(info);
        } catch (const std::runtime_error& e) {
            cached_metadata.clear();
            NODELET_ERROR_STREAM(
                "Error when running in replay mode: " << e.what());
        }
    }

    void allocate_buffers() {
        auto& pf = sensor::get_format(info);

        lidar_packet.buf.resize(pf.lidar_packet_size);
        // TODO: gauge necessary queue size for lidar packets
        lidar_packets =
            std::make_unique<ThreadSafeRingBuffer>(pf.lidar_packet_size, 1024);

        imu_packet.buf.resize(pf.imu_packet_size);
        // TODO: gauge necessary queue size for lidar packets
        imu_packets =
            std::make_unique<ThreadSafeRingBuffer>(pf.imu_packet_size, 1024);
    }

    void create_publishers() {
        auto& nh = getNodeHandle();
        lidar_packet_pub = nh.advertise<PacketMsg>("lidar_packets", 1280);
        imu_packet_pub = nh.advertise<PacketMsg>("imu_packets", 100);
    }

    void open_pcap(const std::string& pcap_file) {
        pcap_handle = ouster::sensor_utils::replay_initialize(pcap_file);
        ouster::sensor_utils::replay_reset(*pcap_handle);    // not sure if this is needed
    }

    void start_packet_read_thread() {
        packet_read_active = true;
        packet_read_thread = std::make_unique<std::thread>([this]() {
            auto& pf = sensor::get_format(info);
            while (packet_read_active) {
                read_packets(*pcap_handle, pf);
            }
            NODELET_DEBUG("packet_read_thread done.");
        });
    }

    void stop_packet_read_thread() {
        NODELET_DEBUG("packet_read_thread stopping.");
        if (packet_read_thread->joinable()) {
            packet_read_active = false;
            packet_read_thread->join();
        }
    }

    void start_packet_processing_threads() {
        imu_packets_processing_thread_active = true;
        imu_packets_processing_thread = std::make_unique<std::thread>([this]() {
            while (imu_packets_processing_thread_active) {
                imu_packets->read([this](const uint8_t* buffer) {
                    on_imu_packet_msg(buffer);
                });
            }
            NODELET_DEBUG("imu_packets_processing_thread done.");
        });

        lidar_packets_processing_thread_active = true;
        lidar_packets_processing_thread = std::make_unique<std::thread>([this]() {
            while (lidar_packets_processing_thread_active) {
                lidar_packets->read([this](const uint8_t* buffer) {
                    on_lidar_packet_msg(buffer);
                });
            }

            NODELET_DEBUG("lidar_packets_processing_thread done.");
        });
    }

    void stop_packet_processing_threads() {
        NODELET_DEBUG("stopping packet processing threads.");

        if (imu_packets_processing_thread->joinable()) {
            imu_packets_processing_thread_active = false;
            imu_packets_processing_thread->join();
        }

        if (lidar_packets_processing_thread->joinable()) {
            lidar_packets_processing_thread_active = false;
            lidar_packets_processing_thread->join();
        }
    }

    void on_lidar_packet_msg(const uint8_t* raw_lidar_packet) {
        // copying the data from queue buffer into the message buffer
        // this can be avoided by constructing an abstraction where
        // OusterSensor has its own RingBuffer of PacketMsg but for
        // now we are focusing on optimizing the code for OusterDriver
        std::memcpy(lidar_packet.buf.data(), raw_lidar_packet,
                    lidar_packet.buf.size());
        lidar_packet_pub.publish(lidar_packet);
    }

    void on_imu_packet_msg(const uint8_t* raw_imu_packet) {
        // copying the data from queue buffer into the message buffer
        // this can be avoided by constructing an abstraction where
        // OusterSensor has its own RingBuffer of PacketMsg but for
        // now we are focusing on optimizing the code for OusterDriver
        std::memcpy(imu_packet.buf.data(), raw_imu_packet, imu_packet.buf.size());
        imu_packet_pub.publish(imu_packet);
    }

    void read_packets(ouster::sensor_utils::playback_handle& handle,
                     const sensor::packet_format& pf) {

        // Buffer to store raw packet data
        ouster::sensor::LidarPacket packet(pf.lidar_packet_size);
        ouster::sensor_utils::packet_info packet_info;
        auto fps = ouster::sensor::frequency_of_lidar_mode(info.mode);
        auto n_cols = ouster::sensor::n_cols_of_lidar_mode(info.mode);
        auto n_packets = n_cols / pf.columns_per_packet;
        auto ts_spacing = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / (fps * n_packets)));


        // TODO: restructure code to permit peeking into next packet before attempting to read
        // without having to rely on sensor hz
        while (ouster::sensor_utils::next_packet_info(handle, packet_info)) {

            if (packet_info.dst_port == info.config.udp_port_imu) {

                    imu_packets->write_overwrite(
                    [this, &handle, &pf, &packet_info](uint8_t* buffer) {

                    auto packet_size = ouster::sensor_utils::read_packet(
                        handle, buffer, pf.imu_packet_size);

                    if (packet_size == pf.imu_packet_size &&
                        packet_info.dst_port == info.config.udp_port_imu) {
                    } else {
                        ROS_ERROR_STREAM("Inconsistent packet_size=" << packet_size
                            << " vs expected=" << pf.imu_packet_size);
                    }
                });
            } else if (packet_info.dst_port == info.config.udp_port_lidar) {
                lidar_packets->write_overwrite(
                    [this, &handle, &pf, &packet_info](uint8_t* buffer) {

                    auto packet_size = ouster::sensor_utils::read_packet(
                        handle, buffer, pf.lidar_packet_size);

                    // make sure packet is valid
                    if (packet_size == pf.lidar_packet_size &&
                        packet_info.dst_port == info.config.udp_port_lidar) {
                    } else {
                        ROS_ERROR_STREAM("Inconsistent packet_size=" << packet_size
                            << " vs expected=" << pf.lidar_packet_size);
                    }

                });

                // only pace lidar packets (this approximates)
                std::this_thread::sleep_for(ts_spacing);

            }
        }
    }

   private:
    std::shared_ptr<ouster::sensor_utils::playback_handle> pcap_handle;
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    ros::Publisher lidar_packet_pub;
    ros::Publisher imu_packet_pub;
    ros::ServiceServer reset_srv;

    std::unique_ptr<ThreadSafeRingBuffer> lidar_packets;
    std::unique_ptr<ThreadSafeRingBuffer> imu_packets;

    std::atomic<bool> packet_read_active = {false};
    std::unique_ptr<std::thread> packet_read_thread;

    std::atomic<bool> imu_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> imu_packets_processing_thread;

    std::atomic<bool> lidar_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> lidar_packets_processing_thread;

    bool force_sensor_reinit = false;
    bool reset_last_init_id = true;

    bool last_init_id_initialized = false;
    uint32_t last_init_id;

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

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterPcap, nodelet::Nodelet)