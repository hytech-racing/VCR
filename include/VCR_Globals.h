#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* C++ library includes */
#include <array> 

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"
/* Local includes */

/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
