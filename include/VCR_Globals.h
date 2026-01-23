#ifndef VCR_GLOBALS
#define VCR_GLOBALS

#include <etl/singleton.h>

/* C++ library includes */
#include <array> 

/* IO Expander MCP23017 Library */
#include "MCP23017.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"


/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* IO Expander Setup */
using IOExpanderInstance = etl::singleton<MCP23017>;

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;

extern unsigned long pulseCount; // NOLINT

#endif /* VCR_GLOBALS */
