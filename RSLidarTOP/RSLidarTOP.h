/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#ifndef __RSLidarTOP__
#define __RSLidarTOP__

// CRITICAL: Include driver headers BEFORE TouchDesigner headers
// This ensures winsock2.h is included before windows.h
#include "../Common/RSLidarDriverCore.h"
#include "../Common/PCAPRecorder.h"
#include "../Common/PCAPCapture.h"
#include <memory>
#include <atomic>
#include <mutex>

// Now include TouchDesigner headers (which include windows.h)
#include "TOP_CPlusPlusBase.h"

// Import TouchDesigner types into global namespace for compatibility
using namespace TD;

/*
RSLidarTOP - RoboSense LiDAR TOP operator for TouchDesigner

Outputs point cloud data as RGBA texture:
  R channel = X coordinate (normalized)
  G channel = Y coordinate (normalized)
  B channel = Z coordinate (normalized)
  A channel = Intensity (0-255)

Features:
  - Zero-copy data flow from rs_driver
  - Configurable texture resolution
  - Auto-scaling coordinate mapping
  - Real-time point cloud streaming
*/

class RSLidarTOP : public TOP_CPlusPlusBase
{
public:
	RSLidarTOP(const OP_NodeInfo* info, TOP_Context* context);
	virtual ~RSLidarTOP();

	virtual void		getGeneralInfo(TOP_GeneralInfo*, const OP_Inputs*, void*) override;

	virtual void		execute(TOP_Output*, const OP_Inputs*, void*) override;

	virtual void		setupParameters(OP_ParameterManager* manager, void*) override;

	// Info DAT
	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1) override;
	virtual void		getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void* reserved1) override;

	// Info CHOP (for IMU data output)
	virtual int32_t		getNumInfoCHOPChans(void* reserved1) override;
	virtual void		getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void* reserved1) override;

private:
	// LiDAR driver core
	std::unique_ptr<RSLidarDriverCore> lidar_driver_;

	// TOP context for buffer creation
	TOP_Context* top_context_;

	// Connection state
	bool is_active_;
	bool was_active_;

	// LiDAR configuration
	std::string lidar_ip_;
	int msop_port_;
	int difop_port_;
	int imu_port_;
	std::string lidar_type_;

	// PCAP playback configuration
	bool use_pcap_;
	std::string pcap_path_;
	bool pcap_repeat_;
	float pcap_rate_;

	// PCAP recording configuration
	bool record_pcap_;
	std::string record_path_;
	bool was_recording_;

	// PCAP recording objects
	std::shared_ptr<PCAPRecorder> pcap_recorder_;
	std::shared_ptr<PCAPCapture> pcap_capture_;

	// Output configuration
	int output_width_;
	int output_height_;

	// Distance filtering
	float min_distance_;
	float max_distance_;

	// Gravity alignment
	bool align_to_gravity_;
	float gravity_smooth_;  // Smoothing factor for accelerometer (0-1, higher = smoother)

	// Smoothed gravity vector
	struct SmoothedGravity {
		float ax = 0.0f;
		float ay = 0.0f;
		float az = 0.0f;
		bool initialized = false;
	};
	SmoothedGravity smoothed_gravity_;
	std::mutex gravity_mutex_;

	// Manual transform parameters
	float translate_x_;
	float translate_y_;
	float translate_z_;
	float rotate_x_;
	float rotate_y_;
	float rotate_z_;

	// Statistics
	std::atomic<uint32_t> execute_count_;
	std::atomic<uint32_t> cloud_count_;
	std::atomic<uint32_t> last_cloud_size_;
	std::atomic<uint32_t> last_texture_width_;
	std::atomic<uint32_t> last_texture_height_;

	// Performance metrics (in microseconds)
	std::atomic<uint64_t> last_execute_time_us_;
	std::atomic<uint64_t> last_get_cloud_time_us_;
	std::atomic<uint64_t> last_conversion_time_us_;
	std::atomic<uint64_t> last_upload_time_us_;
	std::atomic<uint64_t> avg_execute_time_us_;
	std::atomic<uint32_t> perf_sample_count_;
	std::atomic<uint32_t> last_transformed_points_;

	// IMU data for Info CHOP output
	std::mutex imu_data_mutex_;
	std::shared_ptr<ImuData> latest_imu_data_;

	// IMU sensor fusion state
	struct ImuFusionState {
		// Fused orientation quaternion
		float qw = 1.0f;
		float qx = 0.0f;
		float qy = 0.0f;
		float qz = 0.0f;

		// Previous timestamp for integration
		double last_timestamp = 0.0;

		// Complementary filter weight (0-1, higher = trust gyro more)
		float alpha = 0.98f;

		// Initialization flag
		bool initialized = false;
	};
	ImuFusionState imu_fusion_;
	std::mutex imu_fusion_mutex_;

	// Helper methods
	void updateImuFusion(const ImuData& imu_data);
	void initializeDriver();
	void shutdownDriver();
	void updateParameters(const OP_Inputs* inputs);
	void convertPointCloudToRGBA32F(const std::shared_ptr<PointCloudMsg>& cloud,
	                                 float* output_buffer,
	                                 int width, int height);
};

#endif
