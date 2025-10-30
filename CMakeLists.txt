cmake_minimum_required(VERSION 3.15)
project(RSLidarTouchDesigner VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Only allow 64-bit builds (TouchDesigner requirement)
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "TouchDesigner requires 64-bit builds. Please use -A x64 with Visual Studio generator.")
endif()

# Set output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Find rs_driver headers
set(RS_DRIVER_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../rs_driver/src")
if(NOT EXISTS "${RS_DRIVER_INCLUDE_DIR}/rs_driver/api/lidar_driver.hpp")
    message(FATAL_ERROR "rs_driver not found at ${RS_DRIVER_INCLUDE_DIR}. Please clone rs_driver to ../rs_driver/")
endif()

# Common include directories
set(COMMON_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/Common
    ${RS_DRIVER_INCLUDE_DIR}
)

# Platform specific settings
if(WIN32)
    add_definitions(-D_WINDOWS -D_USRDLL -D_CRT_SECURE_NO_WARNINGS)
    # Disable rs_driver PCAP parsing (we use our own PCAP capture)
    add_definitions(-DDISABLE_PCAP_PARSE)

    # Find Npcap SDK (check local project directory first, then system install)
    set(NPCAP_SDK_LOCAL "${CMAKE_CURRENT_SOURCE_DIR}/../npcap-sdk")
    set(NPCAP_SDK_SYSTEM "C:/Program Files/Npcap/SDK")

    if(EXISTS "${NPCAP_SDK_LOCAL}/Include/pcap.h")
        set(NPCAP_DIR "${NPCAP_SDK_LOCAL}")
        message(STATUS "Found Npcap SDK (local) at ${NPCAP_DIR}")
    elseif(EXISTS "${NPCAP_SDK_SYSTEM}/Include/pcap.h")
        set(NPCAP_DIR "${NPCAP_SDK_SYSTEM}")
        message(STATUS "Found Npcap SDK (system) at ${NPCAP_DIR}")
    else()
        message(WARNING "Npcap SDK not found. PCAP recording will be disabled.")
        message(WARNING "Download from https://npcap.com/dist/npcap-sdk-1.15.zip and extract to ${NPCAP_SDK_LOCAL}")
        set(NPCAP_DIR "")
    endif()

    if(NPCAP_DIR)
        set(PCAP_INCLUDE_DIR "${NPCAP_DIR}/Include")
        set(PCAP_LIBRARY "${NPCAP_DIR}/Lib/x64/wpcap.lib")
        set(PACKET_LIBRARY "${NPCAP_DIR}/Lib/x64/Packet.lib")
    else()
        set(PCAP_INCLUDE_DIR "")
        set(PCAP_LIBRARY "")
        set(PACKET_LIBRARY "")
    endif()

    set(PLATFORM_LIBS ws2_32 ${PCAP_LIBRARY} ${PACKET_LIBRARY})
endif()

#
# RSLidarDriverCore (static library)
#
add_library(RSLidarDriverCore STATIC
    Common/RSLidarDriverCore.h
    Common/RSLidarDriverCore.cpp
    Common/RSLidarDriverManager.h
    Common/RSLidarDriverManager.cpp
    Common/PCAPRecorder.h
    Common/PCAPRecorder.cpp
    Common/PCAPCapture.h
    Common/PCAPCapture.cpp
)

target_include_directories(RSLidarDriverCore PUBLIC
    ${COMMON_INCLUDE_DIRS}
    ${PCAP_INCLUDE_DIR}
)

target_link_libraries(RSLidarDriverCore PUBLIC
    ${PLATFORM_LIBS}
)

#
# RSLidarTOP (DLL)
#
add_library(RSLidarTOP SHARED
    RSLidarTOP/RSLidarTOP.h
    RSLidarTOP/RSLidarTOP.cpp
    RSLidarTOP/TOP_CPlusPlusBase.h
)

target_include_directories(RSLidarTOP PRIVATE
    ${COMMON_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/RSLidarTOP
)

target_link_libraries(RSLidarTOP PRIVATE
    RSLidarDriverCore
    ${PLATFORM_LIBS}
)

target_compile_definitions(RSLidarTOP PRIVATE
    RSLIDARTOP_EXPORTS
    _WINSOCKAPI_
)

# Force include winsock2 before any other headers (fixes Windows header conflicts)
if(MSVC)
    target_compile_options(RSLidarTOP PRIVATE
        /FI"winsock2.h"
        /FI"ws2tcpip.h"
    )
endif()

# Set output name
set_target_properties(RSLidarTOP PROPERTIES
    OUTPUT_NAME "RSLidarTOP"
    SUFFIX ".dll"
)

#
# RSLidarCHOP (DLL)
#
# TODO: Re-enable when CHOP functionality is ready
# add_library(RSLidarCHOP SHARED
#     RSLidarCHOP/RSLidarCHOP.h
#     RSLidarCHOP/RSLidarCHOP.cpp
#     RSLidarCHOP/CHOP_CPlusPlusBase.h
# )
# 
# target_include_directories(RSLidarCHOP PRIVATE
#     ${COMMON_INCLUDE_DIRS}
#     ${CMAKE_CURRENT_SOURCE_DIR}/RSLidarCHOP
# )
# 
# target_link_libraries(RSLidarCHOP PRIVATE
#     RSLidarDriverCore
#     ${PLATFORM_LIBS}
# )
# 
# target_compile_definitions(RSLidarCHOP PRIVATE
#     RSLIDARCHOP_EXPORTS
#     _WINSOCKAPI_
# )
# 
# # Force include winsock2 before any other headers
# if(MSVC)
#     target_compile_options(RSLidarCHOP PRIVATE
#         /FI"winsock2.h"
#         /FI"ws2tcpip.h"
#     )
# endif()
# 
# # Set output name
# set_target_properties(RSLidarCHOP PROPERTIES
#     OUTPUT_NAME "RSLidarCHOP"
#     SUFFIX ".dll"
# )
# 
# #
# Installation (optional)
#
if(DEFINED ENV{TOUCHDESIGNER_INSTALL_DIR})
    set(TD_PLUGIN_DIR "$ENV{TOUCHDESIGNER_INSTALL_DIR}/bin/Plugins")
else()
    set(TD_PLUGIN_DIR "C:/Program Files/Derivative/TouchDesigner/bin/Plugins")
endif()

install(TARGETS RSLidarTOP  # RSLidarCHOP
    RUNTIME DESTINATION "${TD_PLUGIN_DIR}"
    LIBRARY DESTINATION "${TD_PLUGIN_DIR}"
)

# Print configuration summary
message(STATUS "")
message(STATUS "========================================")
message(STATUS "RoboSense LiDAR TouchDesigner Integration")
message(STATUS "========================================")
message(STATUS "Build type:         ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ compiler:       ${CMAKE_CXX_COMPILER}")
message(STATUS "C++ standard:       ${CMAKE_CXX_STANDARD}")
message(STATUS "rs_driver include:  ${RS_DRIVER_INCLUDE_DIR}")
message(STATUS "TouchDesigner dir:  ${TD_PLUGIN_DIR}")
message(STATUS "========================================")
message(STATUS "")
message(STATUS "To build:")
message(STATUS "  cmake --build . --config Release")
message(STATUS "")
message(STATUS "To install:")
message(STATUS "  cmake --install . --config Release")
message(STATUS "")
