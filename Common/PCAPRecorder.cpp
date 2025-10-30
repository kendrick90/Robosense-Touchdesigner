/*********************************************************************************************************************
* PCAP Recorder Implementation
*********************************************************************************************************************/

#include "PCAPRecorder.h"
#include <cstring>
#include <iostream>

PCAPRecorder::PCAPRecorder()
    : msop_port_(0)
    , difop_port_(0)
    , imu_port_(0)
    , is_recording_(false)
    , msop_packet_count_(0)
    , difop_packet_count_(0)
    , imu_packet_count_(0)
{
}

PCAPRecorder::~PCAPRecorder()
{
    stopRecording();
}

bool PCAPRecorder::startRecording(const std::string& base_path,
                                   uint16_t msop_port,
                                   uint16_t difop_port,
                                   uint16_t imu_port)
{
    if (is_recording_.load())
    {
        std::cerr << "Already recording!" << std::endl;
        return false;
    }

    // Store port numbers
    msop_port_ = msop_port;
    difop_port_ = difop_port;
    imu_port_ = imu_port;

    // Reset counters
    msop_packet_count_.store(0);
    difop_packet_count_.store(0);
    imu_packet_count_.store(0);

    // Open MSOP file
    std::string msop_path = base_path + "_msop.pcap";
    msop_file_.open(msop_path, std::ios::binary | std::ios::trunc);
    if (!msop_file_.is_open())
    {
        std::cerr << "Failed to open MSOP file: " << msop_path << std::endl;
        return false;
    }
    writePCAPHeader(msop_file_);

    // Open DIFOP file
    std::string difop_path = base_path + "_difop.pcap";
    difop_file_.open(difop_path, std::ios::binary | std::ios::trunc);
    if (!difop_file_.is_open())
    {
        std::cerr << "Failed to open DIFOP file: " << difop_path << std::endl;
        msop_file_.close();
        return false;
    }
    writePCAPHeader(difop_file_);

    // Open IMU file (if enabled)
    if (imu_port_ > 0)
    {
        std::string imu_path = base_path + "_imu.pcap";
        imu_file_.open(imu_path, std::ios::binary | std::ios::trunc);
        if (!imu_file_.is_open())
        {
            std::cerr << "Failed to open IMU file: " << imu_path << std::endl;
            msop_file_.close();
            difop_file_.close();
            return false;
        }
        writePCAPHeader(imu_file_);
    }

    // Record start time
    start_time_ = std::chrono::high_resolution_clock::now();

    is_recording_.store(true);
    std::cout << "Started PCAP recording:" << std::endl;
    std::cout << "  MSOP:  " << msop_path << std::endl;
    std::cout << "  DIFOP: " << difop_path << std::endl;
    if (imu_port_ > 0)
    {
        std::cout << "  IMU:   " << base_path << "_imu.pcap" << std::endl;
    }

    return true;
}

void PCAPRecorder::stopRecording()
{
    if (!is_recording_.load())
        return;

    is_recording_.store(false);

    if (msop_file_.is_open())
        msop_file_.close();
    if (difop_file_.is_open())
        difop_file_.close();
    if (imu_file_.is_open())
        imu_file_.close();

    std::cout << "Stopped PCAP recording. Packets written:" << std::endl;
    std::cout << "  MSOP:  " << msop_packet_count_.load() << std::endl;
    std::cout << "  DIFOP: " << difop_packet_count_.load() << std::endl;
    if (imu_port_ > 0)
    {
        std::cout << "  IMU:   " << imu_packet_count_.load() << std::endl;
    }
}

void PCAPRecorder::writePacket(const uint8_t* data, size_t length,
                               uint16_t src_port, uint16_t dst_port)
{
    if (!is_recording_.load())
        return;

    // Determine which file to write to based on port
    if (dst_port == msop_port_)
    {
        std::lock_guard<std::mutex> lock(msop_mutex_);
        writePCAPPacket(msop_file_, data, length, src_port, dst_port);
        msop_packet_count_.fetch_add(1);
    }
    else if (dst_port == difop_port_)
    {
        std::lock_guard<std::mutex> lock(difop_mutex_);
        writePCAPPacket(difop_file_, data, length, src_port, dst_port);
        difop_packet_count_.fetch_add(1);
    }
    else if (imu_port_ > 0 && dst_port == imu_port_)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        writePCAPPacket(imu_file_, data, length, src_port, dst_port);
        imu_packet_count_.fetch_add(1);
    }
}

void PCAPRecorder::writePCAPHeader(std::ofstream& file)
{
    PCAPFileHeader header;
    header.magic_number = 0xa1b2c3d4;
    header.version_major = 2;
    header.version_minor = 4;
    header.thiszone = 0;
    header.sigfigs = 0;
    header.snaplen = 65535;
    header.network = 1;  // ETHERNET

    file.write(reinterpret_cast<const char*>(&header), sizeof(header));
}

void PCAPRecorder::writePCAPPacket(std::ofstream& file, const uint8_t* data, size_t length,
                                   uint16_t src_port, uint16_t dst_port)
{
    // Calculate timestamp relative to start
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_);
    uint32_t ts_sec = static_cast<uint32_t>(duration.count() / 1000000);
    uint32_t ts_usec = static_cast<uint32_t>(duration.count() % 1000000);

    // Construct Ethernet header
    EthernetHeader eth_hdr;
    memset(eth_hdr.dst_mac, 0xFF, 6);  // Broadcast MAC
    memset(eth_hdr.src_mac, 0x00, 6);  // Dummy source MAC
    eth_hdr.ether_type = htons(0x0800);  // IPv4

    // Construct IPv4 header
    IPv4Header ip_hdr;
    memset(&ip_hdr, 0, sizeof(ip_hdr));
    ip_hdr.version_ihl = 0x45;  // Version 4, IHL 5 (20 bytes)
    ip_hdr.tos = 0;
    ip_hdr.total_length = htons(static_cast<uint16_t>(20 + 8 + length));  // IP + UDP + data
    ip_hdr.identification = 0;
    ip_hdr.flags_fragment = 0;
    ip_hdr.ttl = 64;
    ip_hdr.protocol = 17;  // UDP
    ip_hdr.src_ip = htonl(0xC0A80101);  // 192.168.1.1
    ip_hdr.dst_ip = htonl(0xC0A80102);  // 192.168.1.2
    ip_hdr.checksum = 0;
    ip_hdr.checksum = calculateIPChecksum(&ip_hdr);

    // Construct UDP header
    UDPHeader udp_hdr;
    udp_hdr.src_port = htons(src_port);
    udp_hdr.dst_port = htons(dst_port);
    udp_hdr.length = htons(static_cast<uint16_t>(8 + length));  // UDP header + data
    udp_hdr.checksum = 0;  // Optional for IPv4

    // Calculate total packet size
    size_t total_size = sizeof(EthernetHeader) + sizeof(IPv4Header) + sizeof(UDPHeader) + length;

    // Write PCAP packet header
    PCAPPacketHeader pkt_hdr;
    pkt_hdr.ts_sec = ts_sec;
    pkt_hdr.ts_usec = ts_usec;
    pkt_hdr.incl_len = static_cast<uint32_t>(total_size);
    pkt_hdr.orig_len = static_cast<uint32_t>(total_size);
    file.write(reinterpret_cast<const char*>(&pkt_hdr), sizeof(pkt_hdr));

    // Write Ethernet header
    file.write(reinterpret_cast<const char*>(&eth_hdr), sizeof(eth_hdr));

    // Write IPv4 header
    file.write(reinterpret_cast<const char*>(&ip_hdr), sizeof(ip_hdr));

    // Write UDP header
    file.write(reinterpret_cast<const char*>(&udp_hdr), sizeof(udp_hdr));

    // Write payload
    file.write(reinterpret_cast<const char*>(data), length);

    // Flush to ensure data is written
    file.flush();
}

uint16_t PCAPRecorder::calculateIPChecksum(const IPv4Header* header)
{
    uint32_t sum = 0;
    const uint16_t* data = reinterpret_cast<const uint16_t*>(header);

    // Sum all 16-bit words in header
    for (size_t i = 0; i < sizeof(IPv4Header) / 2; i++)
    {
        sum += ntohs(data[i]);
    }

    // Fold 32-bit sum to 16 bits
    while (sum >> 16)
    {
        sum = (sum & 0xFFFF) + (sum >> 16);
    }

    return htons(static_cast<uint16_t>(~sum));
}
