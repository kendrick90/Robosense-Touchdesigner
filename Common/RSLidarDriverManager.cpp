/*********************************************************************************************************************
* RoboSense LiDAR Driver Manager Implementation
*********************************************************************************************************************/

#include "RSLidarDriverManager.h"

RSLidarDriverManager& RSLidarDriverManager::getInstance()
{
    static RSLidarDriverManager instance;
    return instance;
}

std::shared_ptr<RSLidarDriverCore> RSLidarDriverManager::getDriver(const RSDriverParam& param)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // If driver already exists and is connected, return it
    if (driver_ && driver_->isConnected())
    {
        return driver_;
    }

    // Create new driver instance
    driver_ = std::make_shared<RSLidarDriverCore>();

    // Initialize with provided parameters
    if (driver_->init(param))
    {
        driver_->start();
        return driver_;
    }

    // Failed to initialize
    driver_.reset();
    return nullptr;
}

bool RSLidarDriverManager::isDriverActive() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return driver_ && driver_->isConnected();
}

std::shared_ptr<RSLidarDriverCore> RSLidarDriverManager::getCurrentDriver() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return driver_;
}

void RSLidarDriverManager::releaseDriver()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (driver_)
    {
        driver_->stop();
        driver_.reset();
    }
}
