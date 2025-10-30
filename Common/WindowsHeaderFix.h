// WindowsHeaderFix.h - Fixes Windows/Winsock header ordering conflicts
// Include this BEFORE any TouchDesigner or rs_driver headers
#pragma once

#ifdef _WIN32
    // Must define these BEFORE windows.h is included
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN  // Excludes winsock.h from windows.h
    #endif

    #ifndef NOMINMAX
        #define NOMINMAX  // Prevents min/max macro pollution
    #endif

    // Include winsock2.h FIRST (before windows.h)
    #include <winsock2.h>
    #include <ws2tcpip.h>

    // Now windows.h can be included safely (it won't include winsock.h again)
    // TouchDesigner headers will include windows.h, but it's now safe
#endif
