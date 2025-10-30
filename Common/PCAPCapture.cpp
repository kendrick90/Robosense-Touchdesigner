/*********************************************************************************************************************
* PCAP Capture Implementation using WinPcap/Npcap
*********************************************************************************************************************/

#include "PCAPCapture.h"
#include <iostream>
#include <sstream>

// Include WinPcap/Npcap headers
#define HAVE_REMOTE
#include <pcap.h>

// Pragma to link with wpcap.lib (WinPcap) and ws2_32.lib
#pragma comment(lib, "wpcap.lib")
#pragma comment(lib, "ws2_32.lib")

// Ethernet header structure
struct EthernetHeader {
    uint8_t dst_mac[6];
    uint8_t src_mac[6];
    uint16_t ether_type;
};

// IPv4 header structure
struct IPv4Header {
    uint8_t version_ihl;
    uint8_t tos;
    uint16_t total_length;
    uint16_t identification;
    uint16_t flags_fragment;
    uint8_t ttl;
    uint8_t protocol;
    uint16_t checksum;
    uint32_t src_ip;
    uint32_t dst_ip;
};

// UDP header structure
struct UDPHeader {
    uint16_t src_port;
    uint16_t dst_port;
    uint16_t length;
    uint16_t checksum;
};

PCAPCapture::PCAPCapture()
    : pcap_handle_(nullptr)
    , msop_port_(0)
    , difop_port_(0)
    , imu_port_(0)
    , is_capturing_(false)
    , should_stop_(false)
    , total_packets_(0)
    , total_bytes_(0)
{
}

PCAPCapture::~PCAPCapture()
{
    stopCapture();
}

std::vector<std::string> PCAPCapture::listInterfaces()
{
    std::vector<std::string> interfaces;
    pcap_if_t* alldevs;
    char errbuf[PCAP_ERRBUF_SIZE];

    // Retrieve the device list
    if (pcap_findalldevs(&alldevs, errbuf) == -1)
    {
        std::cerr << "Error finding network interfaces: " << errbuf << std::endl;
        return interfaces;
    }

    // Build interface list
    int i = 0;
    for (pcap_if_t* d = alldevs; d != nullptr; d = d->next)
    {
        std::ostringstream oss;
        oss << i << ": " << (d->description ? d->description : d->name);
        interfaces.push_back(oss.str());
        i++;
    }

    pcap_freealldevs(alldevs);
    return interfaces;
}

bool PCAPCapture::startCapture(int interface_index,
                                     uint16_t msop_port,
                                     uint16_t difop_port,
                                     uint16_t imu_port)
{
    if (is_capturing_.load())
    {
        std::cerr << "PCAPCapture: Already capturing!" << std::endl;
        return false;
    }

    // Store port numbers
    msop_port_ = msop_port;
    difop_port_ = difop_port;
    imu_port_ = imu_port;

    // Get list of devices
    pcap_if_t* alldevs;
    char errbuf[PCAP_ERRBUF_SIZE];

    if (pcap_findalldevs(&alldevs, errbuf) == -1)
    {
        std::cerr << "PCAPCapture: Error finding devices: " << errbuf << std::endl;
        return false;
    }

    // Find the specified interface
    pcap_if_t* selected_dev = nullptr;
    int i = 0;
    for (pcap_if_t* d = alldevs; d != nullptr; d = d->next)
    {
        if (i == interface_index)
        {
            selected_dev = d;
            break;
        }
        i++;
    }

    if (!selected_dev)
    {
        std::cerr << "PCAPCapture: Interface index " << interface_index << " not found!" << std::endl;
        pcap_freealldevs(alldevs);
        return false;
    }

    // Open the device for capturing
    // Snaplen: 65536 (max packet size)
    // Promiscuous mode: 1 (enabled)
    // Read timeout: 1000 ms
    pcap_handle_ = pcap_open_live(selected_dev->name, 65536, 1, 1000, errbuf);

    std::string dev_name = selected_dev->description ? selected_dev->description : selected_dev->name;
    pcap_freealldevs(alldevs);

    if (!pcap_handle_)
    {
        std::cerr << "PCAPCapture: Unable to open device: " << errbuf << std::endl;
        return false;
    }

    // Build filter string for UDP ports
    std::ostringstream filter_exp;
    filter_exp << "udp and (port " << msop_port << " or port " << difop_port;
    if (imu_port > 0)
    {
        filter_exp << " or port " << imu_port;
    }
    filter_exp << ")";

    // Compile and apply the filter
    struct bpf_program fp;
    if (pcap_compile(pcap_handle_, &fp, filter_exp.str().c_str(), 0, PCAP_NETMASK_UNKNOWN) == -1)
    {
        std::cerr << "PCAPCapture: Couldn't parse filter: " << pcap_geterr(pcap_handle_) << std::endl;
        pcap_close(pcap_handle_);
        pcap_handle_ = nullptr;
        return false;
    }

    if (pcap_setfilter(pcap_handle_, &fp) == -1)
    {
        std::cerr << "PCAPCapture: Couldn't install filter: " << pcap_geterr(pcap_handle_) << std::endl;
        pcap_freecode(&fp);
        pcap_close(pcap_handle_);
        pcap_handle_ = nullptr;
        return false;
    }

    pcap_freecode(&fp);

    // Reset counters
    total_packets_.store(0);
    total_bytes_.store(0);

    // Start capture thread
    should_stop_.store(false);
    is_capturing_.store(true);
    capture_thread_ = std::thread(&PCAPCapture::captureThread, this);

    std::cout << "PCAPCapture: Started capturing on: " << dev_name << std::endl;
    std::cout << "  Filter: " << filter_exp.str() << std::endl;

    return true;
}

void PCAPCapture::stopCapture()
{
    if (!is_capturing_.load())
        return;

    // Signal thread to stop
    should_stop_.store(true);

    // Break the pcap_loop
    if (pcap_handle_)
    {
        pcap_breakloop(pcap_handle_);
    }

    // Wait for thread to finish
    if (capture_thread_.joinable())
    {
        capture_thread_.join();
    }

    // Close pcap handle
    if (pcap_handle_)
    {
        pcap_close(pcap_handle_);
        pcap_handle_ = nullptr;
    }

    is_capturing_.store(false);

    std::cout << "PCAPCapture: Stopped. Captured " << total_packets_.load()
              << " packets (" << total_bytes_.load() << " bytes)" << std::endl;
}

void PCAPCapture::captureThread()
{
    struct pcap_pkthdr* header;
    const uint8_t* pkt_data;
    int res;

    while (!should_stop_.load())
    {
        // Get next packet
        res = pcap_next_ex(pcap_handle_, &header, &pkt_data);

        if (res == 1)  // Packet read successfully
        {
            // Parse Ethernet + IP + UDP headers to extract payload
            if (header->caplen < sizeof(EthernetHeader) + 20 + 8)  // Ethernet + IP (min) + UDP
                continue;

            const EthernetHeader* eth_hdr = reinterpret_cast<const EthernetHeader*>(pkt_data);
            uint16_t ether_type = ntohs(eth_hdr->ether_type);

            // Check if IPv4
            if (ether_type != 0x0800)
                continue;

            const IPv4Header* ip_hdr = reinterpret_cast<const IPv4Header*>(pkt_data + sizeof(EthernetHeader));
            uint8_t ip_header_len = (ip_hdr->version_ihl & 0x0F) * 4;

            // Check if UDP
            if (ip_hdr->protocol != 17)
                continue;

            const UDPHeader* udp_hdr = reinterpret_cast<const UDPHeader*>(
                pkt_data + sizeof(EthernetHeader) + ip_header_len);

            uint16_t src_port = ntohs(udp_hdr->src_port);
            uint16_t dst_port = ntohs(udp_hdr->dst_port);
            uint16_t udp_length = ntohs(udp_hdr->length);

            // Extract UDP payload
            const uint8_t* udp_payload = reinterpret_cast<const uint8_t*>(udp_hdr) + 8;
            size_t udp_payload_len = udp_length - 8;  // Subtract UDP header size

            // Write to PCAP recorder if active
            if (pcap_recorder_ && pcap_recorder_->isRecording())
            {
                pcap_recorder_->writePacket(udp_payload, udp_payload_len, src_port, dst_port);

                // Update statistics
                total_packets_.fetch_add(1);
                total_bytes_.fetch_add(udp_payload_len);
            }
        }
        else if (res == 0)  // Timeout
        {
            continue;
        }
        else if (res == -1)  // Error
        {
            if (!should_stop_.load())
            {
                std::cerr << "PCAPCapture: Error reading packet: " << pcap_geterr(pcap_handle_) << std::endl;
            }
            break;
        }
        else if (res == -2)  // pcap_breakloop called
        {
            break;
        }
    }
}
