#include "VCR_Globals.h"
#include "VCR_Constants.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Interface and system data structs */
// VCRInterfaceData_s interface_data;
VCRData_s vcr_data;

/* ADC setup */
MCP_ADC<channels_within_mcp_adc> adc_0 = MCP_ADC<8>(ADC0_CS); // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
MCP_ADC<channels_within_mcp_adc> adc_1 = MCP_ADC<8>(ADC1_CS); // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.