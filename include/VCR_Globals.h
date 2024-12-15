#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Interface and system data structs */
extern VCRInterfaceData_s interface_data;
extern VCRSystemData_s system_data;

/* ADC setup */
<<<<<<< HEAD
extern MCP_ADC<8> adc_0; // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
extern MCP_ADC<8> adc_1; // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.
=======
MCP_ADC<8> adc_0 = MCP_ADC<8>(ADC0_CS); // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
MCP_ADC<8> adc_1 = MCP_ADC<8>(ADC1_CS); // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.
>>>>>>> cb1b012 (Switched buzzer reference to use singleton pattern instead of global instance)

#endif /* VCR_GLOBALS */