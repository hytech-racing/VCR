#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"

/* Interface and system data structs */
extern VCRInterfaceData_s interface_data;
extern VCRSystemData_s system_data;

/* ADC setup */
extern MCP_ADC<8> adc_0; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
extern MCP_ADC<8> adc_1; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.

#endif /* VCR_GLOBALS */