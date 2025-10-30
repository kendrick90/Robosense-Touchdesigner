/*********************************************************************************************************************
* PCAP Capture for RoboSense LiDAR
*
* Captures UDP packets on specified ports using WinPcap/Npcap
* and forwards them to the PCAP recorder for saving to disk.
*
* NOTE: Requires WinPcap or Npcap to be installed on the system.
*       Download from: https://npcap.com/
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
// Forward declare pcap types to avoid requiring pcap.h in header
struct pcap;
typedef struct pcap pcap_t;
#endif

#include "PCAPRecorder.h"
#include <thread>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief PCAP capture for LiDAR data streams using WinPcap/Npcap
 *
 * Captures UDP packets on specified ports and writes them to PCAP files.
 * Much more efficient than raw sockets and provides better filtering.
 */
class PCAPCapture
{
public:
    PCAPCapture();
    ~PCAPCapture();

    /**
     * @brief List available network interfaces
     * @return Vector of interface descriptions (for UI selection)
     */
    static std::vector<std::string> listInterfaces();

    /**
     * @brief Start capturing packets
     * @param interface_index Index of network interface (0-based, from listInterfaces())
     * @param msop_port MSOP port to capture (default 6699)
     * @param difop_port DIFOP port to capture (default 7788)
     * @param imu_port IMU port to capture (default 6688, 0 to disable)
     * @return true if sniffer started successfully
     */
    bool startCapture(int interface_index = 0,
                     uint16_t msop_port = 6699,
                     uint16_t difop_port = 7788,
                     uint16_t imu_port = 6688);

    /**
     * @brief Stop capturing packets
     */
    void stopCapture();

    /**
     * @brief Check if currently capturing
     */
    bool isCapturing() const { return is_capturing_.load(); }

    /**
     * @brief Set the PCAP recorder to write captured packets
     */
    void setPCAPRecorder(std::shared_ptr<PCAPRecorder> recorder) { pcap_recorder_ = recorder; }

    /**
     * @brief Get capture statistics
     */
    uint64_t getTotalPacketsCaptured() const { return total_packets_.load(); }
    uint64_t getTotalBytesCaptured() const { return total_bytes_.load(); }

private:
    // Capture thread function
    void captureThread();

    // WinPcap/Npcap handle
    pcap_t* pcap_handle_;

    // Ports to capture
    uint16_t msop_port_;
    uint16_t difop_port_;
    uint16_t imu_port_;

    // Capture state
    std::atomic<bool> is_capturing_;
    std::atomic<bool> should_stop_;

    // Capture thread
    std::thread capture_thread_;

    // PCAP recorder
    std::shared_ptr<PCAPRecorder> pcap_recorder_;

    // Statistics
    std::atomic<uint64_t> total_packets_;
    std::atomic<uint64_t> total_bytes_;
};
