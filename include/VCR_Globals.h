#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"

/* Interface and system data structs */
extern VCRInterfaceData_s interface_data; // NOLINT
extern VCRSystemData_s system_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
extern MCP_ADC<channels_within_mcp_adc> adc_0; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
extern MCP_ADC<channels_within_mcp_adc> adc_1; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.

/* Watchdog pin */
extern const int watchdog_pin;

#endif /* VCR_GLOBALS */