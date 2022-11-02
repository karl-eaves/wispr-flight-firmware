#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "Wispr Flight Firmware V0.3.2"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 0,3,2,FIRMWARE_VERSION_TYPE_OFFICIAL

#define FW_MAJOR 0
#define FW_MINOR 3
#define FW_PATCH 2
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

