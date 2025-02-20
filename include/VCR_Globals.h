#ifndef VCR_GLOBALS
#define VCR_GLOBALS

#include "etl/singleton.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"
/* Local includes */
#include "VCR_Constants.h"

/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
using ADC0Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
using ADC1Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.


#endif /* VCR_GLOBALS */