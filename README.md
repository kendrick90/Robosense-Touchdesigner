# RoboSense LiDAR TouchDesigner Integration

Complete zero-copy integration between RoboSense LiDAR hardware (via rs_driver) and TouchDesigner using custom C++ operators.

## Quick Start

**What you need to build:**

- Visual Studio 2019/2022 (Windows only)
- TouchDesigner 2023 or later
- RoboSense LiDAR (RS-Airy, RS-LiDAR-M1, etc.)

**What you get:**

- **RSLidarTOP** - Point cloud as RGBA texture (RGB=XYZ, A=Reflectivity) + IMU data via Info CHOP

## Table of Contents

1. [Overview](#overview)
2. [Building from Source](#building-from-source)
3. [Installation](#installation)
4. [Usage in TouchDesigner](#usage-in-touchdesigner)
5. [Examples](#examples)
6. [Troubleshooting](#troubleshooting)
7. [Architecture](#architecture)

---

## Overview

### Features

**RSLidarTOP:**

- Point cloud → RGBA texture (R=X, G=Y, B=Z, A=Reflectivity)
- Zero-copy streaming from rs_driver
- Coordinate transformation (Airy LiDAR → TouchDesigner frame)
- IMU extrinsic calibration support (from DIFOP packets)
- Manual transform parameters (translation + rotation)
- Real-time updates at LiDAR frame rate (10-20 Hz)
- Info CHOP output with 15 IMU channels:
  - Orientation quaternion (from driver)
  - Angular velocity (gyroscope)
  - Linear acceleration (accelerometer)
  - Fused orientation quaternion (sensor fusion)
  - Timestamp
- Info DAT with statistics and performance metrics

**Core Architecture:**

- Thread-safe queue-based architecture
- Zero-copy data transfer via shared pointers
- Automatic memory management
- Device status and error reporting

### System Architecture

```
┌─────────────────────────────────────────────────────────┐
│           RoboSense LiDAR Hardware                      │
└──────────────────────┬──────────────────────────────────┘
                       │ UDP (MSOP/DIFOP/IMU)
┌──────────────────────▼──────────────────────────────────┐
│                 rs_driver Library                        │
│  - MSOP decoder (point cloud)                           │
│  - DIFOP decoder (device info, IMU extrinsic)           │
│  - IMU packet decoder (accel, gyro)                     │
└──────────────┬─────────────────────┬────────────────────┘
               │ Zero-copy Callbacks │
┌──────────────▼─────────────────────▼────────────────────┐
│           RSLidarDriverCore Wrapper                      │
│  ┌──────────────┐           ┌──────────────┐           │
│  │ Cloud Queues │           │ IMU Queues   │           │
│  └──────────────┘           └──────────────┘           │
└──────────────────────────┬──────────────────────────────┘
                           │
                  ┌────────▼────────┐
                  │  RSLidarTOP     │
                  │  - Point Cloud  │
                  │  - IMU Data     │
                  │  - Transforms   │
                  └─────────────────┘
```

---

## Building from Source

### Prerequisites

1. **Visual Studio 2019 or 2022**

   - Install "Desktop development with C++" workload
   - Windows 10/11 SDK required

2. **CMake 3.15+** (optional, for CMake build)

   - Download: https://cmake.org/download/

3. **RoboSense rs_driver** (already included as submodule/external)
   - Located at: `./rs_driver/`

### Build Steps (CMake)

```bash
cd RSLidarTouchDesigner
mkdir build
cd build

# Generate Visual Studio solution
cmake .. -G "Visual Studio 17 2022" -A x64

# Build
cmake --build . --config Release

# Output will be in:
# bin/Release/RSLidarTOP.dll
```

### Build Steps (Visual Studio Manual)

If CMake fails, you can create a solution manually:

1. **Create Solution**: File → New → Project → Blank Solution
2. **Add Projects**:

   - Add RSLidarDriverCore (Static Library)
   - Add RSLidarTOP (DLL)

3. **Configure Each Project**:

**Platform:** x64 (Required!)

**Include Directories:**

```
../../rs_driver/src
../Common
.
```

**Preprocessor Definitions:**

```
_WINDOWS
_USRDLL
DISABLE_PCAP_PARSE
_CRT_SECURE_NO_WARNINGS
WIN32_LEAN_AND_MEAN
NOMINMAX
_WINSOCKAPI_
```

**Additional Libraries:**

```
ws2_32.lib
```

**Forced Include Files (Important for Windows Header Compatibility):**

```
winsock2.h
ws2tcpip.h
```

4. **Build Order**: RSLidarDriverCore → RSLidarTOP

### Known Build Issues - RESOLVED

**✅ Windows Header Conflict (winsock.h vs winsock2.h):**

**Solution Applied:** Include rs_driver headers BEFORE TouchDesigner headers.

In header files:

```cpp
// CRITICAL: Include driver headers FIRST
#include "../Common/RSLidarDriverCore.h"
#include <memory>

// THEN include TouchDesigner headers
#include "CHOP_CPlusPlusBase.h"  // or TOP_CPlusPlusBase.h

// Import TouchDesigner namespace
using namespace TD;
```

This pattern (learned from SlamtecCHOP example) ensures winsock2.h is included before windows.h, preventing type redefinition errors.

---

## Installation

1. Open TouchDesigner
2. Add a `CPlusPlus TOP`
3. In DLL parameter, browse to `RSLidarTOP.dll`
4. Custom parameters should appear

---

## Usage in TouchDesigner

### Network Setup

**Configure your LiDAR:**

1. Set LiDAR destination IP to your computer's IP
2. MSOP Port: `6699` (default)
3. DIFOP Port: `7788` (default)
4. IMU Port: `6688` (if using IMU)

### Using RSLidarTOP

1. Create `CPlusPlus TOP` node
2. Load `RSLidarTOP.dll`
3. Set parameters:

   - **Active**: `On` (to connect)
   - **LiDAR Type**: `RSAIRY` (or your model)
   - **Host Address**: `0.0.0.0` (bind all)
   - **MSOP Port**: `6699` (point cloud)
   - **DIFOP Port**: `7788` (device info)
   - **IMU Port**: `6688` (IMU packets)
   - **Min/Max Distance**: Filter points by range
   - **Translate/Rotate**: Manual transform adjustments

4. **TOP Output** - Point cloud texture (900x96 for RS-Airy):

   - **R** channel = X coordinate (meters, in TouchDesigner frame)
   - **G** channel = Y coordinate (meters, in TouchDesigner frame)
   - **B** channel = Z coordinate (meters, in TouchDesigner frame)
   - **A** channel = Reflectivity (0-255)

5. **Info CHOP Output** - 15 channels of IMU data:

   - `orient_x/y/z/w` - Orientation quaternion (from driver, reserved for future use)
   - `gyro_x/y/z` - Angular velocity (rad/s)
   - `accel_x/y/z` - Linear acceleration (g)
   - `timestamp` - IMU timestamp
   - `fused_quat_x/y/z/w` - Fused orientation from sensor fusion (experimental)

6. **Info DAT Output** - Statistics:
   - Connection status, point cloud count, texture dimensions
   - Performance metrics (execute time, FPS, conversion time)
   - IMU data count, gravity alignment status
   - Transformed points count

### Parameters Reference

| Parameter           | Type   | Default | Description                 |
| ------------------- | ------ | ------- | --------------------------- |
| **Active**          | Toggle | Off     | Connect/disconnect          |
| **LiDAR Type**      | Menu   | RSAIRY  | LiDAR model                 |
| **Host Address**    | String | 0.0.0.0 | Local bind IP               |
| **MSOP Port**       | Int    | 6699    | Point cloud port            |
| **DIFOP Port**      | Int    | 7788    | Device info port            |
| **IMU Port**        | Int    | 6688    | IMU packet port             |
| **Min Distance**    | Float  | 0.1     | Filter min (meters)         |
| **Max Distance**    | Float  | 60.0    | Filter max (meters)         |
| **Translate X/Y/Z** | Float  | 0.0     | Manual translation (meters) |
| **Rotate X/Y/Z**    | Float  | 0.0     | Manual rotation (degrees)   |

---

### Multi-LiDAR Fusion

Configure multiple operators with different ports:

**LiDAR 1:** MSOP=6699, DIFOP=7788
**LiDAR 2:** MSOP=5599, DIFOP=6687

---

### Build Errors

**"Cannot find rs_driver headers":**

- Verify include path: `../../rs_driver/src`
- Check that rs_driver is cloned/present

**"Unresolved external symbols":**

- Ensure `RSLidarDriverCore.cpp` is in project
- Link against `ws2_32.lib`

**"winsock redefinition errors":**

- Ensure forced include: `winsock2.h;ws2tcpip.h`
- Add preprocessor: `_WINSOCKAPI_;WIN32_LEAN_AND_MEAN`
- Follow header ordering pattern (see Build Steps)

### DLL Won't Load in TouchDesigner

1. Check platform is **x64** (not x86)
2. Verify Visual C++ Redistributable 2019+ installed
3. Use `dumpbin /exports RSLidarTOP.dll` to verify exports

---

## Architecture

### Zero-Copy Data Flow

```cpp
// rs_driver requests buffer (callback from driver thread)
std::shared_ptr<PointCloudMsg> getPointCloudCallback() {
    return free_cloud_queue_.pop() ?: std::make_shared<PointCloudMsg>();
}

// rs_driver returns filled buffer (callback from driver thread)
void putPointCloudCallback(std::shared_ptr<PointCloudMsg> cloud) {
    stuffed_cloud_queue_.push(cloud);  // Zero-copy!
}

// TouchDesigner fetches data (main thread)
auto cloud = lidar_driver_->getLatestPointCloud();
// Old cloud automatically returned to free queue
```

### Thread Safety

- **rs_driver callbacks**: Run in driver threads
- **TouchDesigner execute()**: Runs in main thread
- **SyncQueue**: Mutex-protected queue operations
- **Latest data cache**: Mutex-protected atomic updates

## Project Structure

```
RSLidarTouchDesigner/
├── Common/
│   ├── RSLidarDriverCore.h         # Core wrapper header
│   ├── RSLidarDriverCore.cpp       # Core implementation
│   └── CPlusPlus_Common.h          # TD common types
├── RSLidarTOP/
│   ├── RSLidarTOP.h                # TOP plugin header
│   ├── RSLidarTOP.cpp              # Point cloud + IMU implementation
│   └── TOP_CPlusPlusBase.h         # TouchDesigner TOP base class
├── CMakeLists.txt                  # Build configuration
└── README.md (this file)
```

---

## License

This code is subject to the Derivative Inc. Shared Use License for TouchDesigner integration.
rs_driver is licensed under the 3-clause BSD License.

---

## References

- [RoboSense rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)
- [TouchDesigner Custom Operator Samples](https://github.com/TouchDesigner/CustomOperatorSamples)
- [TouchDesigner C++ Plugin Docs](https://docs.derivative.ca/Write_a_CPlusPlus_Plugin)
- [Slamtec CHOP Reference](https://github.com/Ajasra/SlamtecCHOP) (header ordering pattern, TD LiDAR Plugin)
