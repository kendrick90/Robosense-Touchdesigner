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

#include "RSLidarCHOP.h"
#include <algorithm>
#include <cmath>

// DLL exports
extern "C"
{

DLLEXPORT
void
FillCHOPPluginInfo(CHOP_PluginInfo *info)
{
	info->apiVersion = CHOPCPlusPlusAPIVersion;
	info->customOPInfo.opType->setString("Rslidarimu");
	info->customOPInfo.opLabel->setString("RoboSense LiDAR IMU");
	info->customOPInfo.opIcon->setString("RSL");
	info->customOPInfo.authorName->setString("Your Name");
	info->customOPInfo.authorEmail->setString("your@email.com");
	info->customOPInfo.minInputs = 0;
	info->customOPInfo.maxInputs = 0;
}

DLLEXPORT
CHOP_CPlusPlusBase*
CreateCHOPInstance(const OP_NodeInfo* info)
{
	return new RSLidarCHOP(info);
}

DLLEXPORT
void
DestroyCHOPInstance(CHOP_CPlusPlusBase* instance)
{
	delete (RSLidarCHOP*)instance;
}

};

RSLidarCHOP::RSLidarCHOP(const OP_NodeInfo* info)
	: active_(false)
	, buffer_size_(100)
	, imu_count_(0)
	, samples_output_(0)
	, buffer_overruns_(0)
{
}

RSLidarCHOP::~RSLidarCHOP()
{
}

void
RSLidarCHOP::getGeneralInfo(CHOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void*)
{
	ginfo->cookEveryFrame = true;
	ginfo->timeslice = true;  // This CHOP is time-sliced for high-frequency output
	ginfo->inputMatchIndex = 0;
}

bool
RSLidarCHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void*)
{
	info->numChannels = 15;  // 15 IMU channels
	info->numSamples = buffer_size_;  // Output buffered samples
	info->sampleRate = 200;  // 200Hz IMU data
	return true;
}

void
RSLidarCHOP::getChannelName(int32_t index, OP_String* name, const OP_Inputs* inputs, void*)
{
	switch (index)
	{
	case 0: name->setString("orient_x"); break;
	case 1: name->setString("orient_y"); break;
	case 2: name->setString("orient_z"); break;
	case 3: name->setString("orient_w"); break;
	case 4: name->setString("gyro_x"); break;
	case 5: name->setString("gyro_y"); break;
	case 6: name->setString("gyro_z"); break;
	case 7: name->setString("accel_x"); break;
	case 8: name->setString("accel_y"); break;
	case 9: name->setString("accel_z"); break;
	case 10: name->setString("timestamp"); break;
	case 11: name->setString("fused_quat_x"); break;
	case 12: name->setString("fused_quat_y"); break;
	case 13: name->setString("fused_quat_z"); break;
	case 14: name->setString("fused_quat_w"); break;
	default: name->setString(""); break;
	}
}

void
RSLidarCHOP::execute(CHOP_Output* output, const OP_Inputs* inputs, void*)
{
	active_ = inputs->getParInt("Active") != 0;
	buffer_size_ = inputs->getParInt("Buffersize");

	if (!active_)
	{
		// Output zeros when not active
		for (int i = 0; i < output->numChannels; i++)
		{
			for (int j = 0; j < output->numSamples; j++)
			{
				output->channels[i][j] = 0.0f;
			}
		}
		return;
	}

	// Get shared driver instance
	driver_ = RSLidarDriverManager::getInstance().getCurrentDriver();

	if (!driver_ || !driver_->isConnected())
	{
		// No driver available, output zeros
		for (int i = 0; i < output->numChannels; i++)
		{
			for (int j = 0; j < output->numSamples; j++)
			{
				output->channels[i][j] = 0.0f;
			}
		}
		return;
	}

	// Update IMU buffer
	updateIMUBuffer();

	// Output buffered IMU samples
	std::lock_guard<std::mutex> lock(imu_buffer_mutex_);

	int samplesToOutput = (std::min)(static_cast<int>(imu_buffer_.size()), output->numSamples);

	for (int sampleIdx = 0; sampleIdx < samplesToOutput; sampleIdx++)
	{
		const auto& imu = imu_buffer_[sampleIdx];

		output->channels[0][sampleIdx] = imu->orientation_x;
		output->channels[1][sampleIdx] = imu->orientation_y;
		output->channels[2][sampleIdx] = imu->orientation_z;
		output->channels[3][sampleIdx] = imu->orientation_w;
		output->channels[4][sampleIdx] = imu->angular_velocity_x;
		output->channels[5][sampleIdx] = imu->angular_velocity_y;
		output->channels[6][sampleIdx] = imu->angular_velocity_z;
		output->channels[7][sampleIdx] = imu->linear_acceleration_x;
		output->channels[8][sampleIdx] = imu->linear_acceleration_y;
		output->channels[9][sampleIdx] = imu->linear_acceleration_z;
		output->channels[10][sampleIdx] = static_cast<float>(imu->timestamp);
		// Note: fused quaternion would need to be computed/stored separately
		output->channels[11][sampleIdx] = 0.0f;
		output->channels[12][sampleIdx] = 0.0f;
		output->channels[13][sampleIdx] = 0.0f;
		output->channels[14][sampleIdx] = 1.0f;
	}

	// Fill remaining samples with last value
	if (samplesToOutput > 0 && samplesToOutput < output->numSamples)
	{
		for (int i = 0; i < output->numChannels; i++)
		{
			float lastValue = output->channels[i][samplesToOutput - 1];
			for (int j = samplesToOutput; j < output->numSamples; j++)
			{
				output->channels[i][j] = lastValue;
			}
		}
	}

	samples_output_.fetch_add(samplesToOutput);
}

void
RSLidarCHOP::setupParameters(OP_ParameterManager* manager, void*)
{
	{
		OP_NumericParameter np;
		np.name = "Active";
		np.label = "Active";
		np.defaultValues[0] = 0.0;
		OP_ParAppendResult res = manager->appendToggle(np);
	}

	{
		OP_NumericParameter np;
		np.name = "Buffersize";
		np.label = "Buffer Size";
		np.defaultValues[0] = 100;
		np.minSliders[0] = 1;
		np.maxSliders[0] = 1000;
		np.minValues[0] = 1;
		np.maxValues[0] = 1000;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendInt(np);
	}
}

bool
RSLidarCHOP::getInfoDATSize(OP_InfoDATSize* infoSize, void*)
{
	infoSize->rows = 5;
	infoSize->cols = 2;
	infoSize->byColumn = false;
	return true;
}

void
RSLidarCHOP::getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void*)
{
	static char tempBuffer[4096];

	switch (index)
	{
	case 0:
		entries->values[0]->setString("Status");
		entries->values[1]->setString(active_ && driver_ && driver_->isConnected() ? "Connected" : "Disconnected");
		break;
	case 1:
		entries->values[0]->setString("IMU Count");
		sprintf_s(tempBuffer, "%u", imu_count_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 2:
		entries->values[0]->setString("Samples Output");
		sprintf_s(tempBuffer, "%u", samples_output_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 3:
		entries->values[0]->setString("Buffer Size");
		{
			std::lock_guard<std::mutex> lock(imu_buffer_mutex_);
			sprintf_s(tempBuffer, "%zu", imu_buffer_.size());
		}
		entries->values[1]->setString(tempBuffer);
		break;
	case 4:
		entries->values[0]->setString("Buffer Overruns");
		sprintf_s(tempBuffer, "%u", buffer_overruns_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	}
}

void
RSLidarCHOP::updateIMUBuffer()
{
	if (!driver_)
		return;

	// Get all available IMU data from driver
	while (true)
	{
		std::shared_ptr<ImuData> imu = driver_->getLatestIMUData();
		if (!imu)
			break;

		std::lock_guard<std::mutex> lock(imu_buffer_mutex_);

		// Add to buffer
		imu_buffer_.push_back(imu);
		imu_count_.fetch_add(1);

		// Limit buffer size
		if (imu_buffer_.size() > MAX_BUFFER_SIZE)
		{
			imu_buffer_.pop_front();
			buffer_overruns_.fetch_add(1);
		}
	}
}
