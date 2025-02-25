#ifndef VCR_GLOBALS
#define VCR_GLOBALS

#include "etl/singleton.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCR_Constants.h"

/* Interface and system data structs */
extern VCRData_s vcr_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
struct ADCInstances
{
    MCP_ADC <channels_within_mcp_adc> & adc0;
    MCP_ADC <channels_within_mcp_adc> & adc1;
    
    ADCInstances(MCP_ADC <channels_within_mcp_adc> & adc0_, MCP_ADC <channels_within_mcp_adc> & adc1_) : adc0(adc0_), adc1(adc1_) {};
    // ADCInstance() : adc0(adc0_), adc1(adc1_) {}; // Default constructor
};
using ADCSingletonInstance = etl::singleton<ADCInstances>; // Singleton for ADCs. Used to pass ADCs to other systems that need them, such as the TelemetrySystem.


#endif /* VCR_GLOBALS */