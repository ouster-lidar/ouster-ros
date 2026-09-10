/**
 * Copyright (c) 2018-2023, Ouster, Inc.
 * All rights reserved.
 *
 * @file zone_packet_source.h
 * @brief Receives zone monitoring (ZM) packets from a live ouster sensor
 */

#pragma once

#include <ouster/impl/netcompat.h>
#include <ouster/packet.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace ouster_ros {

/**
 * A minimal UDP receiver dedicated to the zone monitoring stream.
 *
 * The sdk client (ouster::sdk::sensor::Client) only binds the lidar and imu
 * data ports, so the driver owns the socket of the zone monitoring port
 * itself. Reads are blocking with a bounded timeout, the intended use is a
 * dedicated read thread.
 */
class ZonePacketSource {
   public:
    /// Result of a read attempt
    enum class ReadStatus { PACKET, TIMEOUT, ERROR };

    /**
     * Bind the zone monitoring port.
     *
     * @param[in] port the zone monitoring port to bind to.
     * @param[in] udp_dest the destination the sensor sends zone monitoring
     * data to, only used to join the group when it is a multicast address.
     * @param[in] mtp_dest address of the host network interface that should
     * join the multicast group, when empty any appropriate interface is used.
     * @param[in] timeout_sec how long a read blocks before reporting TIMEOUT.
     *
     * @throw std::runtime_error when the port could not be bound.
     */
    ZonePacketSource(int port, const std::string& udp_dest,
                     const std::string& mtp_dest, int timeout_sec = 1)
        : sock_fd_(bind_udp_socket(port, udp_dest, mtp_dest, timeout_sec)) {}

    ~ZonePacketSource() {
        if (ouster::sdk::sensor::impl::socket_valid(sock_fd_))
            ouster::sdk::sensor::impl::socket_close(sock_fd_);
    }

    ZonePacketSource(const ZonePacketSource&) = delete;
    ZonePacketSource& operator=(const ZonePacketSource&) = delete;

    /**
     * Read a single zone monitoring packet, blocking up to the configured
     * timeout. The packet buffer needs to be sized to the expected zone packet
     * size, packets of a different size are rejected.
     *
     * @param[out] packet the packet to populate, its host_timestamp attribute
     * is set as well.
     *
     * @return the outcome of the read attempt.
     */
    ReadStatus read(ouster::sdk::core::ZonePacket& packet) {
        const auto size = packet.buf.size();
        // recv one byte more than expected so that oversized packets can be
        // told apart from correctly sized ones
        read_buf_.resize(size + 1);
        auto bytes = ::recv(sock_fd_, reinterpret_cast<char*>(read_buf_.data()),
                            static_cast<int>(read_buf_.size()), 0);
        if (bytes == SOCKET_ERROR) {
            return read_timed_out() ? ReadStatus::TIMEOUT : ReadStatus::ERROR;
        }
        if (static_cast<size_t>(bytes) != size) return ReadStatus::ERROR;
        std::memcpy(packet.buf.data(), read_buf_.data(), size);
        packet.host_timestamp = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::system_clock::now().time_since_epoch())
                .count());
        return ReadStatus::PACKET;
    }

   private:
    /// Tells apart an expired SO_RCVTIMEO (or an interrupted call) from an
    /// actual socket failure.
    static bool read_timed_out() {
#ifdef _WIN32
        auto error = WSAGetLastError();
        return error == WSAETIMEDOUT || error == WSAEWOULDBLOCK ||
               error == WSAEINTR;
#else
        return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR;
#endif
    }

    static SOCKET bind_udp_socket(int port, const std::string& udp_dest,
                                  const std::string& mtp_dest,
                                  int timeout_sec) {
        namespace netimpl = ouster::sdk::sensor::impl;

        struct addrinfo hints;
        std::memset(&hints, 0, sizeof(hints));
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_flags = AI_PASSIVE;

        struct addrinfo* info_start = nullptr;
        auto port_str = std::to_string(port);
        int ret = getaddrinfo(nullptr, port_str.c_str(), &hints, &info_start);
        if (ret != 0 || info_start == nullptr) {
            throw std::runtime_error(
                "zone monitoring socket getaddrinfo(): " +
                std::string(gai_strerror(ret)));
        }

        SOCKET sock_fd = SOCKET_ERROR;
        for (auto* ai = info_start; ai != nullptr; ai = ai->ai_next) {
            sock_fd = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
            if (!netimpl::socket_valid(sock_fd)) continue;

            if (ai->ai_family == AF_INET6) {
                int off = 0;
                setsockopt(sock_fd, IPPROTO_IPV6, IPV6_V6ONLY,
                           reinterpret_cast<char*>(&off), sizeof(off));
            }
            if (netimpl::socket_set_reuse(sock_fd) != 0 ||
                ::bind(sock_fd, ai->ai_addr,
                       static_cast<socklen_t>(ai->ai_addrlen)) == SOCKET_ERROR) {
                netimpl::socket_close(sock_fd);
                sock_fd = SOCKET_ERROR;
                continue;
            }
            break;
        }
        freeaddrinfo(info_start);

        if (!netimpl::socket_valid(sock_fd)) {
            throw std::runtime_error(
                "failed to bind the zone monitoring port " + port_str + ": " +
                netimpl::socket_get_error());
        }

        join_multicast_group(sock_fd, udp_dest, mtp_dest);

        if (netimpl::socket_set_rcvtimeout(sock_fd, timeout_sec) != 0) {
            auto error = netimpl::socket_get_error();
            netimpl::socket_close(sock_fd);
            throw std::runtime_error(
                "zone monitoring socket setsockopt(SO_RCVTIMEO): " + error);
        }

        return sock_fd;
    }

    static void join_multicast_group(SOCKET sock_fd,
                                     const std::string& udp_dest,
                                     const std::string& mtp_dest) {
        if (udp_dest.empty()) return;

        struct ip_mreq mreq;
        std::memset(&mreq, 0, sizeof(mreq));
        if (inet_pton(AF_INET, udp_dest.c_str(), &mreq.imr_multiaddr) != 1)
            return;  // not an ipv4 address, nothing to join
        if (!IN_MULTICAST(ntohl(mreq.imr_multiaddr.s_addr))) return;

        if (mtp_dest.empty()) {
            mreq.imr_interface.s_addr = htonl(INADDR_ANY);
        } else if (inet_pton(AF_INET, mtp_dest.c_str(), &mreq.imr_interface) !=
                   1) {
            throw std::runtime_error(
                "invalid multicast interface address for the zone monitoring "
                "socket: " + mtp_dest);
        }

        if (setsockopt(sock_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                       reinterpret_cast<char*>(&mreq),
                       sizeof(mreq)) == SOCKET_ERROR) {
            auto error = ouster::sdk::sensor::impl::socket_get_error();
            ouster::sdk::sensor::impl::socket_close(sock_fd);
            throw std::runtime_error(
                "zone monitoring socket setsockopt(IP_ADD_MEMBERSHIP): " +
                error);
        }
    }

   private:
    SOCKET sock_fd_;
    std::vector<uint8_t> read_buf_;
};

}  // namespace ouster_ros
