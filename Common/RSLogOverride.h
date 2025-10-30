/*********************************************************************************************************************
* RS_Driver Log Override - Disable ANSI Color Codes
*
* This header must be included BEFORE any rs_driver headers to override the default logging macros
* and remove ANSI escape codes that don't render properly in TouchDesigner's console.
*********************************************************************************************************************/

#pragma once

#include <iostream>

// Override rs_driver logging macros to remove ANSI color codes
// These must be defined before including any rs_driver headers

#ifndef RS_ERROR
#define RS_ERROR   std::cout
#define RS_WARNING std::cout
#define RS_INFO    std::cout
#define RS_INFOL   std::cout
#define RS_DEBUG   std::cout
#define RS_REND    std::endl
#define RS_TITLE   std::cout
#define RS_MSG     std::cout
#endif
