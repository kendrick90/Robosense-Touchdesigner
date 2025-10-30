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

// RSLidarTOP.h already includes all necessary headers in correct order
#include "RSLidarTOP.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>
#include <chrono>

// These functions are basic C function, which the DLL loader can find
// much easier than finding a C++ Class.
// The DLLEXPORT prefix is needed so the compile exports these functions from the .dll
// you are creating
extern "C"
{

DLLEXPORT
void
FillTOPPluginInfo(TOP_PluginInfo *info)
{
	// This must always be set to this constant
	info->apiVersion = TOPCPlusPlusAPIVersion;

	// Change this to change the executeMode behavior of this plugin.
	info->executeMode = TOP_ExecuteMode::CPUMem;

	// For more information on OP_CustomOPInfo see CPlusPlus_Common.h
	OP_CustomOPInfo& customInfo = info->customOPInfo;

	// Unique name of the node which starts with an upper case letter, followed by lower case letters or numbers
	customInfo.opType->setString("Rslidar");
	// English readable name
	customInfo.opLabel->setString("RoboSense LiDAR");
	customInfo.opIcon->setString("RSL");
	// Information of the author of the node
	customInfo.authorName->setString("Your Name");
	customInfo.authorEmail->setString("your@email.com");

	// This TOP takes no inputs
	customInfo.minInputs = 0;
	customInfo.maxInputs = 0;
}

DLLEXPORT
TOP_CPlusPlusBase*
CreateTOPInstance(const OP_NodeInfo* info, TOP_Context* context)
{
	// Return a new instance of your class every time this is called.
	// It will be called once per TOP that is using the .dll
	return new RSLidarTOP(info, context);
}

DLLEXPORT
void
DestroyTOPInstance(TOP_CPlusPlusBase* instance, TOP_Context *context)
{
	// Delete the instance here, this will be called when
	// Touch is shutting down, when the TOP using that instance is deleted, or
	// if the TOP loads a different DLL
	delete (RSLidarTOP*)instance;
}

};

RSLidarTOP::RSLidarTOP(const OP_NodeInfo* info, TOP_Context* context)
	: top_context_(context)
	, lidar_driver_(nullptr)
	, is_active_(false)
	, was_active_(false)
	, msop_port_(6699)
	, difop_port_(7788)
	, imu_port_(0)
	, use_pcap_(false)
	, pcap_repeat_(true)
	, pcap_rate_(1.0f)
	, record_pcap_(false)
	, was_recording_(false)
	, output_width_(512)
	, output_height_(512)
	, min_distance_(0.1f)  // RS AIRY min range from driver spec
	, max_distance_(60.0f)  // RS AIRY max range from driver spec
	, align_to_gravity_(false)
	, gravity_smooth_(0.9f)  // Default smoothing factor (0.9 = very smooth)
	, translate_x_(0.0f)
	, translate_y_(0.0f)
	, translate_z_(0.0f)
	, rotate_x_(0.0f)
	, rotate_y_(0.0f)
	, rotate_z_(0.0f)
	, execute_count_(0)
	, cloud_count_(0)
	, last_cloud_size_(0)
	, last_texture_width_(0)
	, last_texture_height_(0)
	, last_execute_time_us_(0)
	, last_get_cloud_time_us_(0)
	, last_conversion_time_us_(0)
	, last_upload_time_us_(0)
	, avg_execute_time_us_(0)
	, perf_sample_count_(0)
	, last_transformed_points_(0)
	, latest_imu_data_(nullptr)
{
	lidar_type_ = "RSAIRY";
	lidar_ip_ = "0.0.0.0";
	pcap_path_ = "";
	record_path_ = "";
}

RSLidarTOP::~RSLidarTOP()
{
	// Stop PCAP recording if active
	if (pcap_capture_)
	{
		pcap_capture_->stopCapture();
		pcap_capture_.reset();
	}
	if (pcap_recorder_)
	{
		pcap_recorder_->stopRecording();
		pcap_recorder_.reset();
	}

	shutdownDriver();
}

void
RSLidarTOP::getGeneralInfo(TOP_GeneralInfo* ginfo, const OP_Inputs* inputs, void*)
{
	// This TOP should cook every frame when active to get new LiDAR data
	ginfo->cookEveryFrameIfAsked = true;
	ginfo->inputSizeIndex = 0;
}

void
RSLidarTOP::updateImuFusion(const ImuData& imu_data)
{
	std::lock_guard<std::mutex> lock(imu_fusion_mutex_);

	// Get accelerometer and gyroscope data
	float ax = imu_data.linear_acceleration_x;
	float ay = imu_data.linear_acceleration_y;
	float az = imu_data.linear_acceleration_z;

	float gx = imu_data.angular_velocity_x;
	float gy = imu_data.angular_velocity_y;
	float gz = imu_data.angular_velocity_z;

	double timestamp = imu_data.timestamp;

	// Initialize on first run
	if (!imu_fusion_.initialized)
	{
		// Initialize quaternion from accelerometer (gravity direction)
		float mag = sqrtf(ax * ax + ay * ay + az * az);
		if (mag > 0.1f)
		{
			// Normalize gravity vector
			ax /= mag;
			ay /= mag;
			az /= mag;

			// Compute initial orientation from gravity
			// Gravity in sensor frame is (ax, ay, az)
			// TouchDesigner uses Y-up convention, so gravity should point to -Y
			// The sensor measures gravity direction, we want to align -Y axis with measured gravity

			// Compute pitch (rotation around X) and roll (rotation around Z) to align Y-down with gravity
			float pitch = atan2f(ax, sqrtf(ay * ay + az * az));  // Tilt forward/back
			float roll = atan2f(-az, -ay);  // Tilt left/right

			// Convert to quaternion (YXZ order for TD convention)
			float cy = cosf(0.0f * 0.5f);  // yaw = 0 initially
			float sy = sinf(0.0f * 0.5f);
			float cp = cosf(pitch * 0.5f);
			float sp = sinf(pitch * 0.5f);
			float cr = cosf(roll * 0.5f);
			float sr = sinf(roll * 0.5f);

			// Quaternion multiplication order: Yaw * Pitch * Roll
			imu_fusion_.qw = cy * cp * cr + sy * sp * sr;
			imu_fusion_.qx = cy * cp * sr - sy * sp * cr;
			imu_fusion_.qy = sy * cp * cr + cy * sp * sr;
			imu_fusion_.qz = sy * cp * sr - cy * sp * cr;

			imu_fusion_.last_timestamp = timestamp;
			imu_fusion_.initialized = true;
		}
		return;
	}

	// Calculate time delta
	double dt = timestamp - imu_fusion_.last_timestamp;
	if (dt <= 0.0 || dt > 1.0)  // Sanity check: reject bad timestamps
	{
		return;
	}
	imu_fusion_.last_timestamp = timestamp;

	// ===== STEP 1: Gyroscope Integration =====
	// Integrate gyroscope angular velocity to update quaternion
	// q_dot = 0.5 * q * [0, gx, gy, gz]
	float half_gx = 0.5f * gx * dt;
	float half_gy = 0.5f * gy * dt;
	float half_gz = 0.5f * gz * dt;

	float qw_new = imu_fusion_.qw - imu_fusion_.qx * half_gx - imu_fusion_.qy * half_gy - imu_fusion_.qz * half_gz;
	float qx_new = imu_fusion_.qx + imu_fusion_.qw * half_gx - imu_fusion_.qz * half_gy + imu_fusion_.qy * half_gz;
	float qy_new = imu_fusion_.qy + imu_fusion_.qz * half_gx + imu_fusion_.qw * half_gy - imu_fusion_.qx * half_gz;
	float qz_new = imu_fusion_.qz - imu_fusion_.qy * half_gx + imu_fusion_.qx * half_gy + imu_fusion_.qw * half_gz;

	// Normalize gyro-integrated quaternion
	float norm_gyro = sqrtf(qw_new * qw_new + qx_new * qx_new + qy_new * qy_new + qz_new * qz_new);
	if (norm_gyro > 0.0f)
	{
		qw_new /= norm_gyro;
		qx_new /= norm_gyro;
		qy_new /= norm_gyro;
		qz_new /= norm_gyro;
	}

	// ===== STEP 2: Accelerometer Correction =====
	// Compute orientation from accelerometer
	float accel_mag = sqrtf(ax * ax + ay * ay + az * az);
	if (accel_mag > 0.1f)
	{
		// Normalize
		ax /= accel_mag;
		ay /= accel_mag;
		az /= accel_mag;

		// Expected gravity in world frame is [0, -1, 0] (Y-down in TouchDesigner)
		// Rotate by current quaternion to get expected gravity in sensor frame
		// v' = q * v * q_conj, but for [0, -1, 0] we can simplify
		float gx_pred = 2.0f * (qx_new * qy_new + qw_new * qz_new);
		float gy_pred = -(qw_new * qw_new - qx_new * qx_new + qy_new * qy_new - qz_new * qz_new);
		float gz_pred = 2.0f * (qy_new * qz_new - qw_new * qx_new);

		// Error: cross product of predicted and measured gravity
		float ex = ay * gz_pred - az * gy_pred;
		float ey = az * gx_pred - ax * gz_pred;
		float ez = ax * gy_pred - ay * gx_pred;

		// Apply correction as a small rotation (complementary filter)
		float correction_gain = (1.0f - imu_fusion_.alpha) * 0.5f;
		float qw_corr = 1.0f;
		float qx_corr = ex * correction_gain * dt;
		float qy_corr = ey * correction_gain * dt;
		float qz_corr = ez * correction_gain * dt;

		// Normalize correction quaternion
		float norm_corr = sqrtf(qw_corr * qw_corr + qx_corr * qx_corr + qy_corr * qy_corr + qz_corr * qz_corr);
		if (norm_corr > 0.0f)
		{
			qw_corr /= norm_corr;
			qx_corr /= norm_corr;
			qy_corr /= norm_corr;
			qz_corr /= norm_corr;
		}

		// Apply correction: q = q_corr * q_gyro
		float qw_final = qw_corr * qw_new - qx_corr * qx_new - qy_corr * qy_new - qz_corr * qz_new;
		float qx_final = qw_corr * qx_new + qx_corr * qw_new + qy_corr * qz_new - qz_corr * qy_new;
		float qy_final = qw_corr * qy_new - qx_corr * qz_new + qy_corr * qw_new + qz_corr * qx_new;
		float qz_final = qw_corr * qz_new + qx_corr * qy_new - qy_corr * qx_new + qz_corr * qw_new;

		// Normalize final quaternion
		float norm_final = sqrtf(qw_final * qw_final + qx_final * qx_final + qy_final * qy_final + qz_final * qz_final);
		if (norm_final > 0.0f)
		{
			imu_fusion_.qw = qw_final / norm_final;
			imu_fusion_.qx = qx_final / norm_final;
			imu_fusion_.qy = qy_final / norm_final;
			imu_fusion_.qz = qz_final / norm_final;
		}
	}
	else
	{
		// No valid accelerometer data, just use gyro integration
		imu_fusion_.qw = qw_new;
		imu_fusion_.qx = qx_new;
		imu_fusion_.qy = qy_new;
		imu_fusion_.qz = qz_new;
	}
}

void
RSLidarTOP::execute(TOP_Output* output, const OP_Inputs* inputs, void*)
{
	auto t_execute_start = std::chrono::high_resolution_clock::now();
	execute_count_.fetch_add(1);

	// Update parameters
	updateParameters(inputs);

	// Handle connection state changes
	if (is_active_ && !was_active_)
	{
		initializeDriver();
		was_active_ = true;
	}
	else if (!is_active_ && was_active_)
	{
		shutdownDriver();
		was_active_ = false;
	}

	// Handle PCAP recording state changes
	if (record_pcap_ && !was_recording_)
	{
		// Start recording
		if (!record_path_.empty())
		{
			// Create recorder
			pcap_recorder_ = std::make_shared<PCAPRecorder>();
			if (pcap_recorder_->startRecording(record_path_,
			                                    static_cast<uint16_t>(msop_port_),
			                                    static_cast<uint16_t>(difop_port_),
			                                    static_cast<uint16_t>(imu_port_)))
			{
				// Create capture and attach recorder
				pcap_capture_ = std::make_shared<PCAPCapture>();
				pcap_capture_->setPCAPRecorder(pcap_recorder_);

				// Start capturing on interface 0 (default interface)
				if (pcap_capture_->startCapture(0,
				                                static_cast<uint16_t>(msop_port_),
				                                static_cast<uint16_t>(difop_port_),
				                                static_cast<uint16_t>(imu_port_)))
				{
					was_recording_ = true;
				}
				else
				{
					// Failed to start capture, clean up
					pcap_recorder_->stopRecording();
					pcap_recorder_.reset();
					pcap_capture_.reset();
				}
			}
			else
			{
				// Failed to start recording
				pcap_recorder_.reset();
			}
		}
	}
	else if (!record_pcap_ && was_recording_)
	{
		// Stop recording
		if (pcap_capture_)
		{
			pcap_capture_->stopCapture();
			pcap_capture_.reset();
		}
		if (pcap_recorder_)
		{
			pcap_recorder_->stopRecording();
			pcap_recorder_.reset();
		}
		was_recording_ = false;
	}

	// If not active or driver not connected, output black texture
	if (!is_active_ || !lidar_driver_ || !lidar_driver_->isConnected())
	{
		TOP_UploadInfo info;
		info.textureDesc.width = output_width_;
		info.textureDesc.height = output_height_;
		info.textureDesc.texDim = OP_TexDim::e2D;
		info.textureDesc.pixelFormat = OP_PixelFormat::RGBA8Fixed;
		info.colorBufferIndex = 0;

		uint64_t byteSize = output_width_ * output_height_ * 4; // RGBA = 4 bytes per pixel
		OP_SmartRef<TOP_Buffer> outbuf = top_context_->createOutputBuffer(byteSize, TOP_BufferFlags::None, nullptr);

		// Fill with black
		memset(outbuf->data, 0, byteSize);

		output->uploadBuffer(&outbuf, info, nullptr);
		return;
	}

	// Get latest point cloud from driver
	auto t_get_cloud_start = std::chrono::high_resolution_clock::now();
	std::shared_ptr<PointCloudMsg> cloud = lidar_driver_->getLatestPointCloud();

	// Also get latest IMU data for Info CHOP output
	std::shared_ptr<ImuData> imu = lidar_driver_->getLatestIMUData();
	if (imu)
	{
		std::lock_guard<std::mutex> lock(imu_data_mutex_);
		latest_imu_data_ = imu;
	}

	auto t_get_cloud_end = std::chrono::high_resolution_clock::now();
	last_get_cloud_time_us_.store(std::chrono::duration_cast<std::chrono::microseconds>(t_get_cloud_end - t_get_cloud_start).count());

	if (!cloud || cloud->points.empty())
	{
		// No data yet, output black
		TOP_UploadInfo info;
		info.textureDesc.width = output_width_;
		info.textureDesc.height = output_height_;
		info.textureDesc.texDim = OP_TexDim::e2D;
		info.textureDesc.pixelFormat = OP_PixelFormat::RGBA8Fixed;
		info.colorBufferIndex = 0;

		uint64_t byteSize = output_width_ * output_height_ * 4;
		OP_SmartRef<TOP_Buffer> outbuf = top_context_->createOutputBuffer(byteSize, TOP_BufferFlags::None, nullptr);

		memset(outbuf->data, 0, byteSize);

		output->uploadBuffer(&outbuf, info, nullptr);
		return;
	}

	// We have point cloud data!
	cloud_count_.fetch_add(1);
	last_cloud_size_.store(static_cast<uint32_t>(cloud->points.size()));

	// RS AIRY has 96 beams - organize point cloud with beams in Y axis
	// Height is fixed at 96 (number of beams)
	// Width is based on expected azimuth samples per rotation (~900 at 10Hz)
	const int RSAIRY_BEAMS = 96;
	const int EXPECTED_WIDTH = 900;  // Expected azimuth samples per rotation at 10Hz

	int actual_height = RSAIRY_BEAMS;

	// Use fixed width for consistent texture size
	// This handles DIFOP packet gaps and slight variations in rotation speed
	int actual_width = EXPECTED_WIDTH;

	// Store for Info DAT
	last_texture_width_.store(actual_width);
	last_texture_height_.store(actual_height);

	// Create output buffer
	TOP_UploadInfo info;
	info.textureDesc.width = actual_width;
	info.textureDesc.height = actual_height;
	info.textureDesc.texDim = OP_TexDim::e2D;
	info.textureDesc.pixelFormat = OP_PixelFormat::RGBA32Float;
	info.colorBufferIndex = 0;

	uint64_t byteSize = actual_width * actual_height * 4 * sizeof(float);  // 4 floats (RGBA) per pixel
	OP_SmartRef<TOP_Buffer> outbuf = top_context_->createOutputBuffer(byteSize, TOP_BufferFlags::None, nullptr);

	// Convert point cloud to RGBA32Float texture
	auto t_conversion_start = std::chrono::high_resolution_clock::now();
	convertPointCloudToRGBA32F(cloud, static_cast<float*>(outbuf->data), actual_width, actual_height);
	auto t_conversion_end = std::chrono::high_resolution_clock::now();
	last_conversion_time_us_.store(std::chrono::duration_cast<std::chrono::microseconds>(t_conversion_end - t_conversion_start).count());

	// Upload to GPU
	auto t_upload_start = std::chrono::high_resolution_clock::now();
	output->uploadBuffer(&outbuf, info, nullptr);
	auto t_upload_end = std::chrono::high_resolution_clock::now();
	last_upload_time_us_.store(std::chrono::duration_cast<std::chrono::microseconds>(t_upload_end - t_upload_start).count());

	// Calculate total execute time
	auto t_execute_end = std::chrono::high_resolution_clock::now();
	uint64_t execute_time = std::chrono::duration_cast<std::chrono::microseconds>(t_execute_end - t_execute_start).count();
	last_execute_time_us_.store(execute_time);

	// Update running average
	uint32_t count = perf_sample_count_.fetch_add(1);
	uint64_t current_avg = avg_execute_time_us_.load();
	uint64_t new_avg = (current_avg * count + execute_time) / (count + 1);
	avg_execute_time_us_.store(new_avg);
}

void
RSLidarTOP::setupParameters(OP_ParameterManager* manager, void*)
{
	// Connection page
	{
		OP_NumericParameter np;
		np.name = "Active";
		np.label = "Active";
		np.defaultValues[0] = 0.0;
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_StringParameter sp;
		sp.name = "Lidartype";
		sp.label = "LiDAR Type";

		const char* names[] = { "RSAIRY", "RS16", "RS32", "RS80", "RS128", "RSM1", "RSM2" };
		const char* labels[] = { "RS-Airy", "RS-16", "RS-32", "RS-80", "RS-128", "RS-M1", "RS-M2" };

		OP_ParAppendResult res = manager->appendMenu(sp, 7, names, labels);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_StringParameter sp;
		sp.name = "Hostaddress";
		sp.label = "Host Address";
		sp.defaultValue = "0.0.0.0";
		OP_ParAppendResult res = manager->appendString(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Msopport";
		np.label = "MSOP Port";
		np.defaultValues[0] = 6699;
		np.minSliders[0] = 1024;
		np.maxSliders[0] = 65535;
		np.minValues[0] = 1;
		np.maxValues[0] = 65535;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Difopport";
		np.label = "DIFOP Port";
		np.defaultValues[0] = 7788;
		np.minSliders[0] = 1024;
		np.maxSliders[0] = 65535;
		np.minValues[0] = 1;
		np.maxValues[0] = 65535;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Imuport";
		np.label = "IMU Port";
		np.defaultValues[0] = 6688;  // RS AIRY IMU port
		np.minSliders[0] = 0;
		np.maxSliders[0] = 65535;
		np.minValues[0] = 0;  // 0 = IMU disabled
		np.maxValues[0] = 65535;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendInt(np);
		assert(res == OP_ParAppendResult::Success);
	}

	/* TODO: Re-enable when PCAP functionality is ready
	// PCAP playback page
	{
		OP_NumericParameter np;
		np.name = "Usepcap";
		np.label = "Use PCAP File";
		np.defaultValues[0] = 0.0;
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_StringParameter sp;
		sp.name = "Pcappath";
		sp.label = "PCAP File Path";
		sp.defaultValue = "";
		OP_ParAppendResult res = manager->appendFile(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Pcaprepeat";
		np.label = "PCAP Repeat";
		np.defaultValues[0] = 1.0;
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Pcaprate";
		np.label = "PCAP Playback Rate";
		np.defaultValues[0] = 1.0;
		np.minSliders[0] = 0.1;
		np.maxSliders[0] = 10.0;
		np.minValues[0] = 0.1;
		np.maxValues[0] = 100.0;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	*/
	// PCAP recording page
	/* TODO: Re-enable when PCAP functionality is ready
	{
		OP_NumericParameter np;
		np.name = "Recordpcap";
		np.label = "Record PCAP";
		np.defaultValues[0] = 0.0;
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_StringParameter sp;
		sp.name = "Recordpath";
		sp.label = "Recording Path";
		sp.defaultValue = "";
		OP_ParAppendResult res = manager->appendFolder(sp);
		assert(res == OP_ParAppendResult::Success);
	}

	*/
	// Distance filter page
	{
		OP_NumericParameter np;
		np.name = "Mindistance";
		np.label = "Min Distance";
		np.defaultValues[0] = 0.1;  // RS AIRY min range from driver spec
		np.minSliders[0] = 0.0;
		np.maxSliders[0] = 100.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Maxdistance";
		np.label = "Max Distance";
		np.defaultValues[0] = 60.0;  // RS AIRY max range from driver spec (60m)
		np.minSliders[0] = 0.1;
		np.maxSliders[0] = 100.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	// Gravity alignment (disabled - not working yet)
	/*
	{
		OP_NumericParameter np;
		np.name = "Aligntogravity";
		np.label = "Align to Gravity";
		np.defaultValues[0] = 0.0;
		OP_ParAppendResult res = manager->appendToggle(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Gravitysmooth";
		np.label = "Gravity Smoothing";
		np.defaultValues[0] = 0.9;
		np.minSliders[0] = 0.0;
		np.maxSliders[0] = 0.99;
		np.minValues[0] = 0.0;
		np.maxValues[0] = 0.99;
		np.clampMins[0] = true;
		np.clampMaxes[0] = true;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}
	*/

	// Transform page
	{
		OP_NumericParameter np;
		np.name = "Tx";
		np.label = "Translate X";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] = 10.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Ty";
		np.label = "Translate Y";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] = 10.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Tz";
		np.label = "Translate Z";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -10.0;
		np.maxSliders[0] = 10.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Rx";
		np.label = "Rotate X";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -180.0;
		np.maxSliders[0] = 180.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Ry";
		np.label = "Rotate Y";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -180.0;
		np.maxSliders[0] = 180.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}

	{
		OP_NumericParameter np;
		np.name = "Rz";
		np.label = "Rotate Z";
		np.defaultValues[0] = 0.0;
		np.minSliders[0] = -180.0;
		np.maxSliders[0] = 180.0;
		OP_ParAppendResult res = manager->appendFloat(np);
		assert(res == OP_ParAppendResult::Success);
	}
}

bool
RSLidarTOP::getInfoDATSize(OP_InfoDATSize* infoSize, void*)
{
	infoSize->rows = 21;  // Added gravity alignment status, IMU orientation, and transformed points
	infoSize->cols = 2;
	infoSize->byColumn = false;
	return true;
}

void
RSLidarTOP::getInfoDATEntries(int32_t index, int32_t nEntries, OP_InfoDATEntries* entries, void*)
{
	static char tempBuffer[4096];

	switch (index)
	{
	case 0:
		entries->values[0]->setString("Status");
		entries->values[1]->setString(is_active_ ? "Connected" : "Disconnected");
		break;
	case 1:
		entries->values[0]->setString("Execute Count");
		sprintf_s(tempBuffer, "%u", execute_count_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 2:
		entries->values[0]->setString("Cloud Count");
		sprintf_s(tempBuffer, "%u", cloud_count_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 3:
		entries->values[0]->setString("Last Cloud Size");
		sprintf_s(tempBuffer, "%u", last_cloud_size_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 4:
		entries->values[0]->setString("LiDAR Type");
		entries->values[1]->setString(lidar_type_.c_str());
		break;
	case 5:
		entries->values[0]->setString("MSOP Port");
		sprintf_s(tempBuffer, "%d", msop_port_);
		entries->values[1]->setString(tempBuffer);
		break;
	case 6:
		entries->values[0]->setString("DIFOP Port");
		sprintf_s(tempBuffer, "%d", difop_port_);
		entries->values[1]->setString(tempBuffer);
		break;
	case 7:
		entries->values[0]->setString("Queue Size");
		if (lidar_driver_)
		{
			sprintf_s(tempBuffer, "%zu", lidar_driver_->getPointCloudQueueSize());
			entries->values[1]->setString(tempBuffer);
		}
		else
		{
			entries->values[1]->setString("N/A");
		}
		break;
	case 8:
		entries->values[0]->setString("Texture Width");
		sprintf_s(tempBuffer, "%u", last_texture_width_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	case 9:
		entries->values[0]->setString("Texture Height");
		sprintf_s(tempBuffer, "%u", last_texture_height_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	// Performance metrics
	case 10:
		entries->values[0]->setString("Last Execute Time (ms)");
		sprintf_s(tempBuffer, "%.3f", last_execute_time_us_.load() / 1000.0);
		entries->values[1]->setString(tempBuffer);
		break;
	case 11:
		entries->values[0]->setString("Avg Execute Time (ms)");
		sprintf_s(tempBuffer, "%.3f", avg_execute_time_us_.load() / 1000.0);
		entries->values[1]->setString(tempBuffer);
		break;
	case 12:
		entries->values[0]->setString("Get Cloud Time (ms)");
		sprintf_s(tempBuffer, "%.3f", last_get_cloud_time_us_.load() / 1000.0);
		entries->values[1]->setString(tempBuffer);
		break;
	case 13:
		entries->values[0]->setString("Conversion Time (ms)");
		sprintf_s(tempBuffer, "%.3f", last_conversion_time_us_.load() / 1000.0);
		entries->values[1]->setString(tempBuffer);
		break;
	case 14:
		entries->values[0]->setString("Upload Time (ms)");
		sprintf_s(tempBuffer, "%.3f", last_upload_time_us_.load() / 1000.0);
		entries->values[1]->setString(tempBuffer);
		break;
	case 15:
	{
		entries->values[0]->setString("Execute FPS");
		uint64_t exec_time = last_execute_time_us_.load();
		if (exec_time > 0)
		{
			sprintf_s(tempBuffer, "%.1f", 1000000.0 / exec_time);
		}
		else
		{
			sprintf_s(tempBuffer, "0.0");
		}
		entries->values[1]->setString(tempBuffer);
		break;
	}
	case 16:
		entries->values[0]->setString("IMU Port");
		sprintf_s(tempBuffer, "%d", imu_port_);
		entries->values[1]->setString(tempBuffer);
		break;
	case 17:
		entries->values[0]->setString("IMU Data Count");
		if (lidar_driver_)
		{
			sprintf_s(tempBuffer, "%u", lidar_driver_->getIMUDataCount());
			entries->values[1]->setString(tempBuffer);
		}
		else
		{
			entries->values[1]->setString("N/A");
		}
		break;
	case 18:
		entries->values[0]->setString("Align to Gravity");
		entries->values[1]->setString(align_to_gravity_ ? "Enabled" : "Disabled");
		break;
	case 19:
	{
		entries->values[0]->setString("IMU Quaternion");
		std::lock_guard<std::mutex> lock(imu_data_mutex_);
		if (latest_imu_data_)
		{
			sprintf_s(tempBuffer, "%.3f, %.3f, %.3f, %.3f",
				latest_imu_data_->orientation_x,
				latest_imu_data_->orientation_y,
				latest_imu_data_->orientation_z,
				latest_imu_data_->orientation_w);
			entries->values[1]->setString(tempBuffer);
		}
		else
		{
			entries->values[1]->setString("No IMU Data");
		}
		break;
	}
	case 20:
		entries->values[0]->setString("Transformed Points");
		sprintf_s(tempBuffer, "%u", last_transformed_points_.load());
		entries->values[1]->setString(tempBuffer);
		break;
	}
}

// Private helper methods

void
RSLidarTOP::initializeDriver()
{
	// Create driver if not exists
	if (!lidar_driver_)
	{
		lidar_driver_ = std::make_unique<RSLidarDriverCore>();
	}

	// Setup parameters
	RSDriverParam param;
	param.lidar_type = strToLidarType(lidar_type_);

	// Configure input type: PCAP file or online LiDAR
	if (use_pcap_ && !pcap_path_.empty())
	{
		param.input_type = InputType::PCAP_FILE;
		param.input_param.pcap_path = pcap_path_;
		param.input_param.pcap_repeat = pcap_repeat_;
		param.input_param.pcap_rate = pcap_rate_;
	}
	else
	{
		param.input_type = InputType::ONLINE_LIDAR;
		param.input_param.host_address = lidar_ip_;
	}

	// Port configuration (used by both PCAP and online modes)
	param.input_param.msop_port = static_cast<uint16_t>(msop_port_);
	param.input_param.difop_port = static_cast<uint16_t>(difop_port_);
	param.input_param.imu_port = static_cast<uint16_t>(imu_port_);

	// Decoder parameters
	param.decoder_param.min_distance = min_distance_;
	param.decoder_param.max_distance = max_distance_;
	param.decoder_param.use_lidar_clock = false;
	param.decoder_param.dense_points = true;

	// Initialize and start
	if (lidar_driver_->init(param))
	{
		lidar_driver_->start();
	}
}

void
RSLidarTOP::shutdownDriver()
{
	if (lidar_driver_)
	{
		lidar_driver_->stop();
	}
}

void
RSLidarTOP::updateParameters(const OP_Inputs* inputs)
{
	is_active_ = inputs->getParInt("Active") != 0;
	min_distance_ = static_cast<float>(inputs->getParDouble("Mindistance"));
	max_distance_ = static_cast<float>(inputs->getParDouble("Maxdistance"));
	// Gravity alignment disabled for now
	// align_to_gravity_ = inputs->getParInt("Aligntogravity") != 0;
	// gravity_smooth_ = static_cast<float>(inputs->getParDouble("Gravitysmooth"));

	translate_x_ = static_cast<float>(inputs->getParDouble("Tx"));
	translate_y_ = static_cast<float>(inputs->getParDouble("Ty"));
	translate_z_ = static_cast<float>(inputs->getParDouble("Tz"));
	rotate_x_ = static_cast<float>(inputs->getParDouble("Rx"));
	rotate_y_ = static_cast<float>(inputs->getParDouble("Ry"));
	rotate_z_ = static_cast<float>(inputs->getParDouble("Rz"));

	lidar_type_ = inputs->getParString("Lidartype");
	lidar_ip_ = inputs->getParString("Hostaddress");
	msop_port_ = inputs->getParInt("Msopport");
	difop_port_ = inputs->getParInt("Difopport");
	imu_port_ = inputs->getParInt("Imuport");

	/* TODO: Re-enable when PCAP functionality is ready
	use_pcap_ = inputs->getParInt("Usepcap") != 0;
	pcap_path_ = inputs->getParString("Pcappath");
	pcap_repeat_ = inputs->getParInt("Pcaprepeat") != 0;
	pcap_rate_ = static_cast<float>(inputs->getParDouble("Pcaprate"));

	record_pcap_ = inputs->getParInt("Recordpcap") != 0;
	record_path_ = inputs->getParString("Recordpath");
	*/
}

void
RSLidarTOP::convertPointCloudToRGBA32F(const std::shared_ptr<PointCloudMsg>& cloud,
                                        float* output_buffer,
                                        int width, int height)
{
	if (!cloud || cloud->points.empty() || !output_buffer)
		return;

	// Clear buffer to zero (black, transparent)
	memset(output_buffer, 0, width * height * 4 * sizeof(float));

	// Get IMU data for gravity alignment if enabled
	// Compute rotation matrix from accelerometer data IN SENSOR FRAME
	float r00 = 1.0f, r01 = 0.0f, r02 = 0.0f;
	float r10 = 0.0f, r11 = 1.0f, r12 = 0.0f;
	float r20 = 0.0f, r21 = 0.0f, r22 = 1.0f;
	bool has_imu = false;

	if (align_to_gravity_)
	{
		std::lock_guard<std::mutex> lock(imu_data_mutex_);
		if (latest_imu_data_)
		{
			// Get raw accelerometer readings in SENSOR FRAME
			// DO NOT apply coordinate transformation yet!
			float ax_raw = latest_imu_data_->linear_acceleration_x;
			float ay_raw = latest_imu_data_->linear_acceleration_y;
			float az_raw = latest_imu_data_->linear_acceleration_z;

			// Apply exponential moving average smoothing to reduce noise
			std::lock_guard<std::mutex> grav_lock(gravity_mutex_);
			if (!smoothed_gravity_.initialized)
			{
				// Initialize with first reading
				smoothed_gravity_.ax = ax_raw;
				smoothed_gravity_.ay = ay_raw;
				smoothed_gravity_.az = az_raw;
				smoothed_gravity_.initialized = true;
			}
			else
			{
				// Exponential moving average: new = alpha * old + (1 - alpha) * current
				// Higher alpha = smoother but more lag
				float alpha = gravity_smooth_;
				smoothed_gravity_.ax = alpha * smoothed_gravity_.ax + (1.0f - alpha) * ax_raw;
				smoothed_gravity_.ay = alpha * smoothed_gravity_.ay + (1.0f - alpha) * ay_raw;
				smoothed_gravity_.az = alpha * smoothed_gravity_.az + (1.0f - alpha) * az_raw;
			}

			// Use smoothed values for gravity alignment IN SENSOR FRAME
			float ax_sensor = smoothed_gravity_.ax;
			float ay_sensor = smoothed_gravity_.ay;
			float az_sensor = smoothed_gravity_.az;

			// Calculate magnitude of acceleration vector
			float mag = sqrtf(ax_sensor * ax_sensor + ay_sensor * ay_sensor + az_sensor * az_sensor);

			// Only proceed if we have valid accelerometer data
			if (mag > 0.1f)  // Threshold to avoid division by zero
			{
				// Normalize the gravity vector in sensor frame
				ax_sensor /= mag;
				ay_sensor /= mag;
				az_sensor /= mag;

				// Now we need to align gravity to the desired direction AFTER coordinate transformation
				// Coordinate transformation: Sensor(X,Y,Z) → TD(Y,X,-Z)
				// So sensor gravity (ax_sensor, ay_sensor, az_sensor) becomes TD gravity (ay_sensor, ax_sensor, -az_sensor)
				// We want to align this to TD Y-down (0, -1, 0)

				// Transformed gravity direction in TD space
				float ax_td = ay_sensor;
				float ay_td = ax_sensor;
				float az_td = -az_sensor;

				// Target gravity direction in TD: Y-down
				float target_x = 0.0f;
				float target_y = -1.0f;  // Y-down for TouchDesigner
				float target_z = 0.0f;

				// Rotation axis = cross(gravity_td, target)
				float axis_x = ay_td * target_z - az_td * target_y;
				float axis_y = az_td * target_x - ax_td * target_z;
				float axis_z = ax_td * target_y - ay_td * target_x;

				// Normalize axis
				float axis_mag = sqrtf(axis_x * axis_x + axis_y * axis_y + axis_z * axis_z);

				if (axis_mag > 0.001f)  // Not already aligned
				{
					axis_x /= axis_mag;
					axis_y /= axis_mag;
					axis_z /= axis_mag;

					// Rotation angle = arccos(dot(gravity_td, target))
					float cos_angle = ax_td * target_x + ay_td * target_y + az_td * target_z;
					// Clamp to valid range for acos [-1, 1] to avoid NaN
					if (cos_angle < -1.0f) cos_angle = -1.0f;
					if (cos_angle > 1.0f) cos_angle = 1.0f;
					float angle = acosf(cos_angle);

					// Build rotation matrix using Rodrigues' formula
					// This rotation is applied IN TD SPACE (after coordinate transformation)
					float c = cosf(angle);
					float s = sinf(angle);
					float t = 1.0f - c;

					r00 = t * axis_x * axis_x + c;
					r01 = t * axis_x * axis_y - s * axis_z;
					r02 = t * axis_x * axis_z + s * axis_y;

					r10 = t * axis_x * axis_y + s * axis_z;
					r11 = t * axis_y * axis_y + c;
					r12 = t * axis_y * axis_z - s * axis_x;

					r20 = t * axis_x * axis_z - s * axis_y;
					r21 = t * axis_y * axis_z + s * axis_x;
					r22 = t * axis_z * axis_z + c;

					has_imu = true;
				}
			}
		}
	}

	// Map points to texture using ring number (beam) for Y coordinate
	// Each point's ring number determines its Y position (0-95 for RS AIRY)
	// X position is determined by order within each ring/beam

	// Track current X position for each ring
	std::vector<int> current_x_per_ring(height, 0);

	uint32_t transformed_count = 0;

	for (const auto& pt : cloud->points)
	{
		// Get beam number (Y coordinate)
		int y = pt.ring;
		if (y < 0 || y >= height)
			continue;

		// Get X position (sequential within this beam)
		int x = current_x_per_ring[y]++;
		if (x >= width)
			continue;

		// STEP 1: Apply coordinate transformation from sensor frame to TD frame
		// Sensor → TouchDesigner mapping:
		// Sensor +Z → TD -Z
		// Sensor X → TD Y
		// Sensor Y → TD X
		float px = pt.y;   // Sensor Y → TD X
		float py = pt.x;   // Sensor X → TD Y
		float pz = -pt.z;  // Sensor +Z → TD -Z

		// STEP 2: Apply gravity alignment rotation (in TD frame)
		// This rotation aligns the transformed gravity vector with Y-down
		if (align_to_gravity_ && has_imu)
		{
			// Apply rotation matrix computed from accelerometer data
			float rx = r00 * px + r01 * py + r02 * pz;
			float ry = r10 * px + r11 * py + r12 * pz;
			float rz = r20 * px + r21 * py + r22 * pz;

			px = rx;
			py = ry;
			pz = rz;

			transformed_count++;
		}

		// Apply manual rotation (Euler angles in degrees, applied in XYZ order)
		if (rotate_x_ != 0.0f || rotate_y_ != 0.0f || rotate_z_ != 0.0f)
		{
			// Convert degrees to radians
			const float PI = 3.14159265358979323846f;
			float rx_rad = rotate_x_ * PI / 180.0f;
			float ry_rad = rotate_y_ * PI / 180.0f;
			float rz_rad = rotate_z_ * PI / 180.0f;

			// Compute rotation matrices
			float cx = cosf(rx_rad), sx = sinf(rx_rad);
			float cy = cosf(ry_rad), sy = sinf(ry_rad);
			float cz = cosf(rz_rad), sz = sinf(rz_rad);

			// Combined rotation matrix (ZYX order)
			float r00 = cy * cz;
			float r01 = sx * sy * cz - cx * sz;
			float r02 = cx * sy * cz + sx * sz;

			float r10 = cy * sz;
			float r11 = sx * sy * sz + cx * cz;
			float r12 = cx * sy * sz - sx * cz;

			float r20 = -sy;
			float r21 = sx * cy;
			float r22 = cx * cy;

			// Apply rotation
			float rx = r00 * px + r01 * py + r02 * pz;
			float ry = r10 * px + r11 * py + r12 * pz;
			float rz = r20 * px + r21 * py + r22 * pz;

			px = rx;
			py = ry;
			pz = rz;
		}

		// Apply manual translation
		px += translate_x_;
		py += translate_y_;
		pz += translate_z_;

		// Store XYZ coordinates as floats
		// R = X, G = Y, B = Z, A = Intensity (normalized to 0-1)
		size_t pixel_index = (y * width + x) * 4;
		output_buffer[pixel_index + 0] = px;  // X coordinate in meters
		output_buffer[pixel_index + 1] = py;  // Y coordinate in meters
		output_buffer[pixel_index + 2] = pz;  // Z coordinate in meters
		output_buffer[pixel_index + 3] = pt.intensity / 255.0f;  // Intensity normalized to 0-1
	}

	// Store transformed count for Info DAT
	last_transformed_points_.store(transformed_count);
}

// Info CHOP implementation for IMU data output

int32_t
RSLidarTOP::getNumInfoCHOPChans(void*)
{
	// Output 15 channels of IMU data:
	// - orientation (quaternion from driver): x, y, z, w (4 channels)
	// - angular_velocity (gyro): x, y, z (3 channels)
	// - linear_acceleration (accel): x, y, z (3 channels)
	// - timestamp (1 channel)
	// - fused_orientation (computed quaternion from sensor fusion): x, y, z, w (4 channels)
	return 15;
}

void
RSLidarTOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* chan, void*)
{
	std::lock_guard<std::mutex> lock(imu_data_mutex_);

	// If no IMU data yet, output zeros
	if (!latest_imu_data_)
	{
		chan->value = 0.0f;
		return;
	}

	// Map channel index to IMU data field
	switch (index)
	{
	// Orientation (quaternion)
	case 0:
		chan->name->setString("orient_x");
		chan->value = latest_imu_data_->orientation_x;
		break;
	case 1:
		chan->name->setString("orient_y");
		chan->value = latest_imu_data_->orientation_y;
		break;
	case 2:
		chan->name->setString("orient_z");
		chan->value = latest_imu_data_->orientation_z;
		break;
	case 3:
		chan->name->setString("orient_w");
		chan->value = latest_imu_data_->orientation_w;
		break;

	// Angular velocity (gyro)
	case 4:
		chan->name->setString("gyro_x");
		chan->value = latest_imu_data_->angular_velocity_x;
		break;
	case 5:
		chan->name->setString("gyro_y");
		chan->value = latest_imu_data_->angular_velocity_y;
		break;
	case 6:
		chan->name->setString("gyro_z");
		chan->value = latest_imu_data_->angular_velocity_z;
		break;

	// Linear acceleration (accel)
	case 7:
		chan->name->setString("accel_x");
		chan->value = latest_imu_data_->linear_acceleration_x;
		break;
	case 8:
		chan->name->setString("accel_y");
		chan->value = latest_imu_data_->linear_acceleration_y;
		break;
	case 9:
		chan->name->setString("accel_z");
		chan->value = latest_imu_data_->linear_acceleration_z;
		break;

	// Timestamp
	case 10:
		chan->name->setString("timestamp");
		chan->value = static_cast<float>(latest_imu_data_->timestamp);
		break;

	// Fused orientation (quaternion from sensor fusion)
	case 11:
		{
			std::lock_guard<std::mutex> lock(imu_fusion_mutex_);
			chan->name->setString("fused_quat_x");
			chan->value = imu_fusion_.qx;
		}
		break;
	case 12:
		{
			std::lock_guard<std::mutex> lock(imu_fusion_mutex_);
			chan->name->setString("fused_quat_y");
			chan->value = imu_fusion_.qy;
		}
		break;
	case 13:
		{
			std::lock_guard<std::mutex> lock(imu_fusion_mutex_);
			chan->name->setString("fused_quat_z");
			chan->value = imu_fusion_.qz;
		}
		break;
	case 14:
		{
			std::lock_guard<std::mutex> lock(imu_fusion_mutex_);
			chan->name->setString("fused_quat_w");
			chan->value = imu_fusion_.qw;
		}
		break;

	default:
		chan->value = 0.0f;
		break;
	}
}
