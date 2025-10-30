/*********************************************************************************************************************
* RoboSense LiDAR Driver Manager
*
* Singleton manager to share a single RSLidarDriverCore instance between multiple operators
* (e.g., RSLidarTOP and RSLidarCHOP) to avoid port conflicts.
*********************************************************************************************************************/

#pragma once

#include "RSLidarDriverCore.h"
#include <memory>
#include <mutex>
#include <string>

/**
 * @brief Singleton manager for shared RSLidarDriverCore instance
 *
 * Allows multiple TouchDesigner operators to share the same LiDAR driver instance
 * without port conflicts. The driver is reference-counted and automatically
 * cleaned up when the last operator releases it.
 */
class RSLidarDriverManager
{
public:
    /**
     * @brief Get the singleton instance
     */
    static RSLidarDriverManager& getInstance();

    /**
     * @brief Get or create the shared driver instance
     * @param param Driver parameters (only used if creating new instance)
     * @return Shared pointer to driver core
     */
    std::shared_ptr<RSLidarDriverCore> getDriver(const RSDriverParam& param);

    /**
     * @brief Check if driver exists and is connected
     */
    bool isDriverActive() const;

    /**
     * @brief Get current driver instance (may be null)
     */
    std::shared_ptr<RSLidarDriverCore> getCurrentDriver() const;

    /**
     * @brief Force release the driver (for cleanup)
     */
    void releaseDriver();

private:
    RSLidarDriverManager() = default;
    ~RSLidarDriverManager() = default;

    // Prevent copying
    RSLidarDriverManager(const RSLidarDriverManager&) = delete;
    RSLidarDriverManager& operator=(const RSLidarDriverManager&) = delete;

    std::shared_ptr<RSLidarDriverCore> driver_;
    mutable std::mutex mutex_;
};
