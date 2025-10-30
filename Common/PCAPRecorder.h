/*********************************************************************************************************************
* PCAP Recorder for RoboSense LiDAR
*
* Records UDP packets from MSOP, DIFOP, and IMU streams to separate PCAP files
* for later playback and analysis.
*********************************************************************************************************************/

#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define _WINSOCKAPI_
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include <string>
#include <fstream>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstdint>
#include <vector>

/**
 * @brief Simple PCAP file writer for UDP packets
 *
 * Writes packets in PCAP format that can be read by Wireshark and rs_driver.
 * Creates separate files for MSOP, DIFOP, and IMU streams.
 */
class PCAPRecorder
{
public:
    PCAPRecorder();
    ~PCAPRecorder();

    /**
     * @brief Start recording to PCAP files
     * @param base_path Base path for PCAP files (e.g., "C:/recordings/lidar")
     *                  Will create: base_path_msop.pcap, base_path_difop.pcap, base_path_imu.pcap
     * @param msop_port MSOP port number (default 6699)
     * @param difop_port DIFOP port number (default 7788)
     * @param imu_port IMU port number (default 6688, 0 to disable)
     * @return true if recording started successfully
     */
    bool startRecording(const std::string& base_path,
                       uint16_t msop_port = 6699,
                       uint16_t difop_port = 7788,
                       uint16_t imu_port = 6688);

    /**
     * @brief Stop recording and close PCAP files
     */
    void stopRecording();

    /**
     * @brief Check if currently recording
     */
    bool isRecording() const { return is_recording_.load(); }

    /**
     * @brief Write a UDP packet to the appropriate PCAP file
     * @param data Packet data
     * @param length Packet length
     * @param src_port Source port
     * @param dst_port Destination port
     */
    void writePacket(const uint8_t* data, size_t length, uint16_t src_port, uint16_t dst_port);

    /**
     * @brief Get statistics
     */
    uint64_t getMSOPPacketCount() const { return msop_packet_count_.load(); }
    uint64_t getDIFOPPacketCount() const { return difop_packet_count_.load(); }
    uint64_t getIMUPacketCount() const { return imu_packet_count_.load(); }

private:
    // PCAP file header structure
    struct PCAPFileHeader {
        uint32_t magic_number;   // 0xa1b2c3d4
        uint16_t version_major;  // 2
        uint16_t version_minor;  // 4
        int32_t  thiszone;       // GMT to local correction
        uint32_t sigfigs;        // accuracy of timestamps
        uint32_t snaplen;        // max length of captured packets
        uint32_t network;        // data link type (1 = ETHERNET)
    };

    // PCAP packet header structure
    struct PCAPPacketHeader {
        uint32_t ts_sec;         // timestamp seconds
        uint32_t ts_usec;        // timestamp microseconds
        uint32_t incl_len;       // number of octets of packet saved in file
        uint32_t orig_len;       // actual length of packet
    };

    // Ethernet header (14 bytes)
    struct EthernetHeader {
        uint8_t  dst_mac[6];     // destination MAC
        uint8_t  src_mac[6];     // source MAC
        uint16_t ether_type;     // 0x0800 for IPv4
    };

    // IPv4 header (20 bytes minimum)
    struct IPv4Header {
        uint8_t  version_ihl;    // version (4 bits) + IHL (4 bits)
        uint8_t  tos;            // type of service
        uint16_t total_length;   // total length
        uint16_t identification; // identification
        uint16_t flags_fragment; // flags (3 bits) + fragment offset (13 bits)
        uint8_t  ttl;            // time to live
        uint8_t  protocol;       // protocol (17 = UDP)
        uint16_t checksum;       // header checksum
        uint32_t src_ip;         // source IP address
        uint32_t dst_ip;         // destination IP address
    };

    // UDP header (8 bytes)
    struct UDPHeader {
        uint16_t src_port;       // source port
        uint16_t dst_port;       // destination port
        uint16_t length;         // length
        uint16_t checksum;       // checksum
    };

    // Helper functions
    void writePCAPHeader(std::ofstream& file);
    void writePCAPPacket(std::ofstream& file, const uint8_t* data, size_t length,
                        uint16_t src_port, uint16_t dst_port);
    uint16_t calculateIPChecksum(const IPv4Header* header);

    // File streams
    std::ofstream msop_file_;
    std::ofstream difop_file_;
    std::ofstream imu_file_;

    // Port numbers
    uint16_t msop_port_;
    uint16_t difop_port_;
    uint16_t imu_port_;

    // Recording state
    std::atomic<bool> is_recording_;

    // Statistics
    std::atomic<uint64_t> msop_packet_count_;
    std::atomic<uint64_t> difop_packet_count_;
    std::atomic<uint64_t> imu_packet_count_;

    // Mutex for thread safety
    std::mutex msop_mutex_;
    std::mutex difop_mutex_;
    std::mutex imu_mutex_;

    // Start time for relative timestamps
    std::chrono::high_resolution_clock::time_point start_time_;
};
