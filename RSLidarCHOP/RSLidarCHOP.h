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

#ifndef __RSLidarCHOP__
#define __RSLidarCHOP__

// CRITICAL: Include driver headers BEFORE TouchDesigner headers
#include "../Common/RSLidarDriverManager.h"
#include <memory>
#include <atomic>
#include <mutex>
#include <deque>

// Now include TouchDesigner headers
#include "CHOP_CPlusPlusBase.h"

// Import TouchDesigner types into global namespace
using namespace TD;

/*
RSLidarCHOP - RoboSense LiDAR IMU CHOP operator for TouchDesigner

Outputs high-frequency IMU data (200Hz) from RoboSense LiDAR sensors.
Shares the same driver instance with RSLidarTOP to avoid port conflicts.

Output channels (15 channels):
  - orient_x, orient_y, orient_z, orient_w (quaternion from driver)
  - gyro_x, gyro_y, gyro_z (angular velocity)
  - accel_x, accel_y, accel_z (linear acceleration)
  - timestamp
  - fused_quat_x, fused_quat_y, fused_quat_z, fused_quat_w (sensor fusion quaternion)
*/

class RSLidarCHOP : public CHOP_CPlusPlusBase
{
public:
	RSLidarCHOP(const OP_NodeInfo* info);
	virtual ~RSLidarCHOP();

	virtual void		getGeneralInfo(CHOP_GeneralInfo*, const OP_Inputs*, void*) override;
	virtual bool		getOutputInfo(CHOP_OutputInfo*, const OP_Inputs*, void*) override;
	virtual void		getChannelName(int32_t index, OP_String *name, const OP_Inputs*, void*) override;

	virtual void		execute(CHOP_Output*, const OP_Inputs*, void*) override;

	virtual void		setupParameters(OP_ParameterManager* manager, void*) override;

	// Info DAT
	virtual bool		getInfoDATSize(OP_InfoDATSize* infoSize, void* reserved1) override;
	virtual void		getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void* reserved1) override;

private:
	// Shared driver instance
	std::shared_ptr<RSLidarDriverCore> driver_;

	// IMU data buffer (for high-frequency output)
	std::deque<std::shared_ptr<ImuData>> imu_buffer_;
	std::mutex imu_buffer_mutex_;
	static constexpr size_t MAX_BUFFER_SIZE = 1000; // ~5 seconds at 200Hz

	// Parameters
	bool active_;
	int buffer_size_;

	// Statistics
	std::atomic<uint32_t> imu_count_;
	std::atomic<uint32_t> samples_output_;
	std::atomic<uint32_t> buffer_overruns_;

	// Helper methods
	void updateIMUBuffer();
};

#endif
