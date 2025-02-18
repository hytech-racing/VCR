#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"
#include "etl/singleton.h"
/* Local includes */
#include "VCR_Constants.h"

/* Interface and system data structs */
// extern VCRInterfaceData_s interface_data; // NOLINT
extern VCRData_s vcr_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
using ADC0Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
using ADC1Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
#endif /* VCR_GLOBALS */