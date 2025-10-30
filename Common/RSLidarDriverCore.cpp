/*********************************************************************************************************************
* RoboSense LiDAR Driver Core for TouchDesigner - Implementation
*********************************************************************************************************************/

#include "RSLidarDriverCore.h"
#include <iostream>

// Static callback wrappers
std::shared_ptr<PointCloudMsg> driverGetPointCloudCallback(void* obj)
{
    return static_cast<RSLidarDriverCore*>(obj)->getPointCloudCallback();
}

void driverPutPointCloudCallback(void* obj, std::shared_ptr<PointCloudMsg> cloud)
{
    static_cast<RSLidarDriverCore*>(obj)->putPointCloudCallback(cloud);
}

std::shared_ptr<ImuData> driverGetIMUDataCallback(void* obj)
{
    return static_cast<RSLidarDriverCore*>(obj)->getIMUDataCallback();
}

void driverPutIMUDataCallback(void* obj, const std::shared_ptr<ImuData>& imu)
{
    static_cast<RSLidarDriverCore*>(obj)->putIMUDataCallback(imu);
}

void driverExceptionCallback(void* obj, const Error& error)
{
    static_cast<RSLidarDriverCore*>(obj)->exceptionCallback(error);
}

RSLidarDriverCore::RSLidarDriverCore()
    : driver_(nullptr)
    , is_connected_(false)
    , point_cloud_count_(0)
    , imu_data_count_(0)
    , latest_point_cloud_(nullptr)
    , latest_imu_data_(nullptr)
{
}

RSLidarDriverCore::~RSLidarDriverCore()
{
    stop();
}

bool RSLidarDriverCore::init(const RSDriverParam& param)
{
    if (is_connected_.load())
    {
        std::cerr << "RSLidarDriverCore: Already connected" << std::endl;
        return false;
    }

    try
    {
        // Create driver instance
        driver_ = std::make_shared<LidarDriver<PointCloudMsg>>();

        // Register point cloud callbacks using lambdas to capture 'this'
        driver_->regPointCloudCallback(
            [this]() { return this->getPointCloudCallback(); },
            [this](std::shared_ptr<PointCloudMsg> cloud) { this->putPointCloudCallback(cloud); }
        );

        // Register IMU data callbacks if IMU port is configured
        if (param.input_param.imu_port > 0)
        {
            driver_->regImuDataCallback(
                [this]() { return this->getIMUDataCallback(); },
                [this](const std::shared_ptr<ImuData>& imu) { this->putIMUDataCallback(imu); }
            );
        }

        // Register exception callback
        driver_->regExceptionCallback(
            [this](const Error& error) { this->exceptionCallback(error); }
        );

        // Initialize driver with parameters
        if (!driver_->init(param))
        {
            std::cerr << "RSLidarDriverCore: Failed to initialize driver" << std::endl;
            driver_ = nullptr;
            return false;
        }

        param.print();
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "RSLidarDriverCore: Exception during init: " << e.what() << std::endl;
        driver_ = nullptr;
        return false;
    }
}

bool RSLidarDriverCore::start()
{
    if (!driver_)
    {
        std::cerr << "RSLidarDriverCore: Driver not initialized" << std::endl;
        return false;
    }

    if (is_connected_.load())
    {
        std::cerr << "RSLidarDriverCore: Already started" << std::endl;
        return true;
    }

    if (driver_->start())
    {
        is_connected_.store(true);
        std::cout << "RSLidarDriverCore: Started successfully" << std::endl;
        return true;
    }

    std::cerr << "RSLidarDriverCore: Failed to start" << std::endl;
    return false;
}

void RSLidarDriverCore::stop()
{
    if (driver_ && is_connected_.load())
    {
        driver_->stop();
        is_connected_.store(false);

        // Clear queues
        free_cloud_queue_.clear();
        stuffed_cloud_queue_.clear();
        free_imu_queue_.clear();
        stuffed_imu_queue_.clear();

        std::cout << "RSLidarDriverCore: Stopped" << std::endl;
    }
}

std::shared_ptr<PointCloudMsg> RSLidarDriverCore::getLatestPointCloud()
{
    // Try to get latest from stuffed queue (non-blocking)
    std::shared_ptr<PointCloudMsg> cloud = stuffed_cloud_queue_.pop();

    if (cloud && cloud.get() != nullptr)
    {
        // Update latest cache
        {
            std::lock_guard<std::mutex> lock(latest_cloud_mutex_);

            // Return old latest to free queue for reuse
            if (latest_point_cloud_ && latest_point_cloud_.get() != nullptr)
            {
                free_cloud_queue_.push(latest_point_cloud_);
            }

            latest_point_cloud_ = cloud;
        }
        return cloud;
    }

    // Return cached latest if no new data
    std::lock_guard<std::mutex> lock(latest_cloud_mutex_);
    return latest_point_cloud_;
}

std::shared_ptr<ImuData> RSLidarDriverCore::getLatestIMUData()
{
    // Try to get latest from stuffed queue (non-blocking)
    std::shared_ptr<ImuData> imu = stuffed_imu_queue_.pop();

    if (imu && imu.get() != nullptr)
    {
        // Update latest cache
        {
            std::lock_guard<std::mutex> lock(latest_imu_mutex_);

            // Return old latest to free queue for reuse
            if (latest_imu_data_ && latest_imu_data_.get() != nullptr)
            {
                free_imu_queue_.push(latest_imu_data_);
            }

            latest_imu_data_ = imu;
        }
        return imu;
    }

    // Return cached latest if no new data
    std::lock_guard<std::mutex> lock(latest_imu_mutex_);
    return latest_imu_data_;
}

bool RSLidarDriverCore::getDeviceInfo(DeviceInfo& info)
{
    if (driver_)
    {
        return driver_->getDeviceInfo(info);
    }
    return false;
}

bool RSLidarDriverCore::getDeviceStatus(DeviceStatus& status)
{
    if (driver_)
    {
        return driver_->getDeviceStatus(status);
    }
    return false;
}

bool RSLidarDriverCore::getTemperature(float& temp)
{
    if (driver_)
    {
        return driver_->getTemperature(temp);
    }
    return false;
}

size_t RSLidarDriverCore::getPointCloudQueueSize()
{
    return stuffed_cloud_queue_.size();
}

size_t RSLidarDriverCore::getIMUDataQueueSize()
{
    return stuffed_imu_queue_.size();
}

// Private callback implementations
std::shared_ptr<PointCloudMsg> RSLidarDriverCore::getPointCloudCallback()
{
    // Note: This runs in rs_driver's thread, DO NOT do time-consuming work here!

    // Try to get from free queue
    std::shared_ptr<PointCloudMsg> cloud = free_cloud_queue_.pop();

    if (cloud.get() != nullptr)
    {
        return cloud;
    }

    // Create new if free queue is empty
    return std::make_shared<PointCloudMsg>();
}

void RSLidarDriverCore::putPointCloudCallback(std::shared_ptr<PointCloudMsg> cloud)
{
    // Note: This runs in rs_driver's thread, DO NOT do time-consuming work here!

    if (cloud && cloud.get() != nullptr)
    {
        stuffed_cloud_queue_.push(cloud);
        point_cloud_count_.fetch_add(1);
    }
}

std::shared_ptr<ImuData> RSLidarDriverCore::getIMUDataCallback()
{
    // Note: This runs in rs_driver's thread, DO NOT do time-consuming work here!

    // Try to get from free queue
    std::shared_ptr<ImuData> imu = free_imu_queue_.pop();

    if (imu.get() != nullptr)
    {
        return imu;
    }

    // Create new if free queue is empty
    return std::make_shared<ImuData>();
}

void RSLidarDriverCore::putIMUDataCallback(const std::shared_ptr<ImuData>& imu)
{
    // Note: This runs in rs_driver's thread, DO NOT do time-consuming work here!

    if (imu && imu.get() != nullptr)
    {
        stuffed_imu_queue_.push(imu);
        imu_data_count_.fetch_add(1);
    }
}

void RSLidarDriverCore::exceptionCallback(const Error& error)
{
    // Note: This runs in rs_driver's thread, DO NOT do time-consuming work here!
    std::cerr << "RSLidarDriverCore Error: " << error.toString() << std::endl;
}
