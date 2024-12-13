#ifndef ADC_INTERFACE
#define ADC_INTERFACE

/* From shared-interfaces-lib libdep */
#include "MCP_ADC.h"

/* ADC setup */
MCP_ADC<8> ADC_0 = MCP_ADC<8>(ADC0_CS); // MCP3208. ADC0 in VCR schematic. Used for valuable telem data.
MCP_ADC<8> ADC_1 = MCP_ADC<8>(ADC1_CS); // MCP3208. ADC1 in VCR schematic. Used for extra thermistors or extra sensors while testing.

#endif /* ADC_INTERFACE */