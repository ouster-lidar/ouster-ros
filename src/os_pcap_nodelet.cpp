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
using ouster::sensor_utils::PcapReader;
using namespace std::chrono;

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
            NODELET_FATAL("Must specify metadata file in replay mode");
            throw std::runtime_error("metadata param not set");
        }
        return meta_file;
    }

    std::string get_pcap_file() const {
        auto pcap_file =
            getPrivateNodeHandle().param("pcap_file", std::string{});
        if (!is_arg_set(pcap_file)) {
            NODELET_FATAL("Must specify pcap file in pcap replay mode");
            throw std::runtime_error("pcap_file param not set");
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
                "Error loading metadata: " << e.what());
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
        pcap.reset(new PcapReader(pcap_file));
    }

    void start_packet_read_thread() {
        packet_read_active = true;
        packet_read_thread = std::make_unique<std::thread>([this]() {
            auto& pf = sensor::get_format(info);
            while (packet_read_active) {
                read_packets(*pcap, pf);
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
        lidar_packets_processing_thread =
            std::make_unique<std::thread>([this]() {
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
        std::memcpy(imu_packet.buf.data(), raw_imu_packet,
                    imu_packet.buf.size());
        imu_packet_pub.publish(imu_packet);
    }

    void read_packets(PcapReader& pcap, const sensor::packet_format& pf) {
        size_t payload_size = pcap.next_packet();
        auto packet_info = pcap.current_info();
        auto file_start = packet_info.timestamp;
        auto last_update = file_start;
        using namespace std::chrono_literals;
        const auto UPDATE_PERIOD = duration_cast<microseconds>(1s / 3);

        while (payload_size) {
            auto start = high_resolution_clock::now();
            if (packet_info.dst_port == info.config.udp_port_imu) {
                imu_packets->write_overwrite(
                    [this, &pcap, &pf, &packet_info](uint8_t* buffer) {
                        std::memcpy(buffer, pcap.current_data(),
                                    pf.imu_packet_size);
                    });
            } else if (packet_info.dst_port == info.config.udp_port_lidar) {
                lidar_packets->write_overwrite(
                    [this, &pcap, &pf, &packet_info](uint8_t* buffer) {
                        std::memcpy(buffer, pcap.current_data(),
                                    pf.lidar_packet_size);
                    });
            } else {
                std::cout << "unknown packet" << std::endl;
            }
            auto prev_packet_ts = packet_info.timestamp;
            payload_size = pcap.next_packet();
            packet_info = pcap.current_info();
            auto curr_packet_ts = packet_info.timestamp;
            auto end = high_resolution_clock::now();
            auto dt = (curr_packet_ts - prev_packet_ts) - (end - start);
            std::this_thread::sleep_for(dt);  // pace packet generation

            if (curr_packet_ts - last_update > UPDATE_PERIOD) {
                last_update = curr_packet_ts;
                std::cout << "\rtime passed: "
                    << std::fixed << std::setprecision(3)
                    << (curr_packet_ts - file_start).count() / 1e6f
                    << " s" << std::flush;
            }
        }
    }

   private:
    std::shared_ptr<PcapReader> pcap;
    PacketMsg lidar_packet;
    PacketMsg imu_packet;
    ros::Publisher lidar_packet_pub;
    ros::Publisher imu_packet_pub;

    std::unique_ptr<ThreadSafeRingBuffer> lidar_packets;
    std::unique_ptr<ThreadSafeRingBuffer> imu_packets;

    std::atomic<bool> packet_read_active = {false};
    std::unique_ptr<std::thread> packet_read_thread;

    std::atomic<bool> imu_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> imu_packets_processing_thread;

    std::atomic<bool> lidar_packets_processing_thread_active = {false};
    std::unique_ptr<std::thread> lidar_packets_processing_thread;
};

}  // namespace ouster_ros

PLUGINLIB_EXPORT_CLASS(ouster_ros::OusterPcap, nodelet::Nodelet)