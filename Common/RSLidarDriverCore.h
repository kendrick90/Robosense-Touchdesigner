/*********************************************************************************************************************
* RoboSense LiDAR Driver Core for TouchDesigner
*
* This class wraps the rs_driver library and provides a zero-copy interface
* for TouchDesigner CHOP and TOP operators.
*********************************************************************************************************************/

#pragma once

// Fix Windows header conflicts - must come before any other includes
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
// Prevent winsock.h from being included by windows.h
#define _WINSOCKAPI_
#include <winsock2.h>
#include <ws2tcpip.h>
#endif
// Override rs_driver logging to remove ANSI color codes
#include "RSLogOverride.h"


#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/msg/imu_data_msg.hpp>
#include <rs_driver/utility/sync_queue.hpp>

#include <memory>
#include <atomic>
#include <string>
#include <mutex>

using namespace robosense::lidar;

// Define point types - Use PointXYZIRT to get ring/beam number
typedef PointXYZIRT PointT;
typedef PointCloudT<PointT> PointCloudMsg;

/**
 * @brief Core wrapper for RoboSense LiDAR driver
 *
 * Manages the lifecycle of the rs_driver and provides thread-safe access
 * to point cloud and IMU data through zero-copy callback queues.
 */
class RSLidarDriverCore
{
public:
    RSLidarDriverCore();
    ~RSLidarDriverCore();

    // Connection management
    bool init(const RSDriverParam& param);
    bool start();
    void stop();
    bool isConnected() const { return is_connected_.load(); }

    // Data access (non-blocking)
    std::shared_ptr<PointCloudMsg> getLatestPointCloud();
    std::shared_ptr<ImuData> getLatestIMUData();

    // Device information
    bool getDeviceInfo(DeviceInfo& info);
    bool getDeviceStatus(DeviceStatus& status);
    bool getTemperature(float& temp);

    // Statistics
    uint32_t getPointCloudCount() const { return point_cloud_count_.load(); }
    uint32_t getIMUDataCount() const { return imu_data_count_.load(); }
    size_t getPointCloudQueueSize();
    size_t getIMUDataQueueSize();

private:
    // rs_driver instance
    std::shared_ptr<LidarDriver<PointCloudMsg>> driver_;

    // Connection state
    std::atomic<bool> is_connected_;
    std::atomic<uint32_t> point_cloud_count_;
    std::atomic<uint32_t> imu_data_count_;

    // Zero-copy queues for point cloud
    SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
    SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;

    // Zero-copy queues for IMU data
    SyncQueue<std::shared_ptr<ImuData>> free_imu_queue_;
    SyncQueue<std::shared_ptr<ImuData>> stuffed_imu_queue_;

    // Latest data cache (for TouchDesigner access)
    std::shared_ptr<PointCloudMsg> latest_point_cloud_;
    std::shared_ptr<ImuData> latest_imu_data_;
    std::mutex latest_cloud_mutex_;
    std::mutex latest_imu_mutex_;

    // Callback functions for rs_driver
    std::shared_ptr<PointCloudMsg> getPointCloudCallback();
    void putPointCloudCallback(std::shared_ptr<PointCloudMsg> cloud);

    std::shared_ptr<ImuData> getIMUDataCallback();
    void putIMUDataCallback(const std::shared_ptr<ImuData>& imu);

    void exceptionCallback(const Error& error);

    // Friend functions for callbacks (to access private members)
    friend std::shared_ptr<PointCloudMsg> driverGetPointCloudCallback(void* obj);
    friend void driverPutPointCloudCallback(void* obj, std::shared_ptr<PointCloudMsg> cloud);
    friend std::shared_ptr<ImuData> driverGetIMUDataCallback(void* obj);
    friend void driverPutIMUDataCallback(void* obj, const std::shared_ptr<ImuData>& imu);
    friend void driverExceptionCallback(void* obj, const Error& error);
};

// Static callback wrappers that forward to instance methods
std::shared_ptr<PointCloudMsg> driverGetPointCloudCallback(void* obj);
void driverPutPointCloudCallback(void* obj, std::shared_ptr<PointCloudMsg> cloud);
std::shared_ptr<ImuData> driverGetIMUDataCallback(void* obj);
void driverPutIMUDataCallback(void* obj, const std::shared_ptr<ImuData>& imu);
void driverExceptionCallback(void* obj, const Error& error);
