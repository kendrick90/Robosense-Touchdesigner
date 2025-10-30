# Dependencies Installation Guide

This guide explains how to set up all required dependencies for building the RoboSense LiDAR TouchDesigner plugin.

## Table of Contents
- [Quick Start](#quick-start)
- [Required Dependencies](#required-dependencies)
- [Optional Dependencies](#optional-dependencies)
- [Troubleshooting](#troubleshooting)

---

## Quick Start

Run these commands in the `New Driver` directory:

```bash
# 1. Clone rs_driver
git clone https://github.com/RoboSense-LiDAR/rs_driver.git

# 2. Download and extract Npcap SDK
curl -L -o npcap-sdk.zip "https://npcap.com/dist/npcap-sdk-1.15.zip"
powershell -Command "Expand-Archive -Path npcap-sdk.zip -DestinationPath npcap-sdk -Force"
rm npcap-sdk.zip

# 3. Build the project
cd RSLidarTouchDesigner/build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
```

---

## Required Dependencies

### 1. **Visual Studio 2022** (or 2019)
**Purpose:** C++ compiler and build tools
**Download:** https://visualstudio.microsoft.com/downloads/
**Required Workloads:**
- Desktop development with C++
- Windows 10/11 SDK

**Verification:**
```bash
"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.38.33130\bin\Hostx64\x64\cl.exe"
```

---

### 2. **CMake 3.15 or higher**
**Purpose:** Build system generator
**Download:** https://cmake.org/download/
**Installation:** Add to PATH during install

**Verification:**
```bash
cmake --version
```

---

### 3. **rs_driver (RoboSense LiDAR SDK)**
**Purpose:** Core LiDAR packet decoding and point cloud generation
**Version:** Latest from main branch
**License:** BSD 3-Clause

**Installation:**
```bash
cd "New Driver"
git clone https://github.com/RoboSense-LiDAR/rs_driver.git
```

**Expected Directory Structure:**
```
New Driver/
├── RSLidarTouchDesigner/
├── rs_driver/              ← Clone here
│   └── src/
│       └── rs_driver/
│           └── api/
│               └── lidar_driver.hpp
```

**Verification:**
The CMake configuration will check for `rs_driver/src/rs_driver/api/lidar_driver.hpp`

**Documentation:** https://github.com/RoboSense-LiDAR/rs_driver

---

### 4. **Npcap SDK** (for PCAP recording/playback)
**Purpose:** Network packet capture for recording LiDAR streams
**Version:** 1.15 or higher
**License:** Free for open source

**Installation Option A - Automatic (Recommended):**
```bash
cd "New Driver"
curl -L -o npcap-sdk.zip "https://npcap.com/dist/npcap-sdk-1.15.zip"
powershell -Command "Expand-Archive -Path npcap-sdk.zip -DestinationPath npcap-sdk -Force"
rm npcap-sdk.zip
```

**Installation Option B - Manual:**
1. Download: https://npcap.com/dist/npcap-sdk-1.15.zip
2. Extract to `New Driver/npcap-sdk/`
3. Verify structure:
   ```
   New Driver/
   ├── npcap-sdk/          ← Extract here
   │   ├── Include/
   │   │   └── pcap.h
   │   └── Lib/
   │       └── x64/
   │           ├── wpcap.lib
   │           └── Packet.lib
   ```

**Runtime Requirement:**
Npcap runtime must be installed on the system to use PCAP features:
- Download: https://npcap.com/dist/npcap-1.80.exe
- Install with "WinPcap API-compatible mode" enabled
- This is separate from the SDK and must be installed even after SDK is extracted

**Verification:**
CMake will automatically detect the SDK and print:
```
-- Found Npcap SDK (local) at .../npcap-sdk
```

---

### 5. **TouchDesigner SDK**
**Purpose:** Plugin API headers (TOP_CPlusPlusBase.h, etc.)
**Version:** Included in project
**License:** Derivative TouchDesigner Shared Use License

**Files Included:**
```
RSLidarTouchDesigner/
└── RSLidarTOP/
    └── TOP_CPlusPlusBase.h  ← Already included
```

No separate installation needed - headers are included in the repository.

---

## Optional Dependencies

### Git (for cloning rs_driver)
**Download:** https://git-scm.com/download/win
**Alternative:** Download rs_driver as ZIP from GitHub

### curl (for downloading Npcap SDK)
**Included in:** Windows 10 1803+ and Windows 11
**Alternative:** Download manually from browser

---

## Directory Structure After Setup

```
Robosense-Airy-Touchdesigner/
└── New Driver/
    ├── RSLidarTouchDesigner/     ← Main plugin source
    │   ├── Common/
    │   │   ├── RSLidarDriverCore.h/cpp
    │   │   ├── PCAPRecorder.h/cpp
    │   │   └── PCAPCapture.h/cpp
    │   ├── RSLidarTOP/
    │   │   └── RSLidarTOP.h/cpp
    │   ├── build/                ← CMake build directory
    │   └── CMakeLists.txt
    │
    ├── rs_driver/                ← Git clone (not committed)
    │   └── src/
    │       └── rs_driver/
    │
    ├── npcap-sdk/                ← Downloaded SDK (not committed)
    │   ├── Include/
    │   └── Lib/
    │
    └── DEPENDENCIES.md           ← This file
```

---

## Build Instructions

After installing all dependencies:

```bash
# Navigate to build directory
cd "RSLidarTouchDesigner/build"

# Configure with CMake (first time only)
cmake .. -G "Visual Studio 17 2022" -A x64

# Build Release version
cmake --build . --config Release

# Or use MSBuild directly
"C:\Program Files\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" ^
    RSLidarTouchDesigner.sln -p:Configuration=Release -p:Platform=x64
```

**Output:**
```
RSLidarTouchDesigner/build/bin/Release/RSLidarTOP.dll
```

---

## Troubleshooting

### CMake can't find rs_driver
**Error:** `rs_driver not found at ...`
**Solution:**
```bash
cd "New Driver"
git clone https://github.com/RoboSense-LiDAR/rs_driver.git
```

### CMake can't find Npcap SDK
**Warning:** `Npcap SDK not found. PCAP recording will be disabled.`
**Solution:**
```bash
cd "New Driver"
curl -L -o npcap-sdk.zip "https://npcap.com/dist/npcap-sdk-1.15.zip"
powershell -Command "Expand-Archive -Path npcap-sdk.zip -DestinationPath npcap-sdk -Force"
```

### PCAP features don't work at runtime
**Error:** "Failed to create raw socket" or "Unable to open device"
**Solution:** Install Npcap runtime from https://npcap.com/dist/npcap-1.80.exe

### Visual Studio version mismatch
**Error:** `error MSB8020: The build tools for v143 cannot be found`
**Solution:**
- Install Visual Studio 2022, OR
- Edit CMakeLists.txt to use your VS version, OR
- Use the VS Developer Command Prompt for your version

### Wrong architecture (x86 vs x64)
**Error:** `LNK1112: module machine type 'x86' conflicts with target machine type 'x64'`
**Solution:** Always use `-A x64` flag:
```bash
cmake .. -G "Visual Studio 17 2022" -A x64
```

---

## Updating Dependencies

### Update rs_driver
```bash
cd "New Driver/rs_driver"
git pull origin main
```

### Update Npcap SDK
Download latest from https://npcap.com/dist/ and extract to `npcap-sdk/`

---

## Offline Installation

If you need to build without internet access:

1. **Download on a machine with internet:**
   - rs_driver: `git clone https://github.com/RoboSense-LiDAR/rs_driver.git`
   - Npcap SDK: https://npcap.com/dist/npcap-sdk-1.15.zip

2. **Transfer to offline machine:**
   - Copy `rs_driver/` folder to `New Driver/rs_driver/`
   - Extract npcap-sdk.zip to `New Driver/npcap-sdk/`

3. **Build normally:**
   ```bash
   cd RSLidarTouchDesigner/build
   cmake .. -G "Visual Studio 17 2022" -A x64
   cmake --build . --config Release
   ```

---

## License Information

- **rs_driver:** BSD 3-Clause License
- **Npcap SDK:** Free for open source, see https://npcap.com/oem/redist.html
- **TouchDesigner SDK:** Derivative Shared Use License
- **This Plugin:** Your license here

---

## Support

For build issues:
1. Check this guide
2. Verify CMake output for dependency detection
3. Check build logs in `RSLidarTouchDesigner/build/`
4. Open an issue with full CMake output and error messages
