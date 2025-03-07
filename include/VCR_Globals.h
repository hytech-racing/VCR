#ifndef VCR_GLOBALS
#define VCR_GLOBALS

/* C++ library includes */
#include <array> 


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
struct ADCBundle
{
    
    //ADCBundle(MCP_ADC <channels_within_mcp_adc> &adc0_, MCP_ADC <channels_within_mcp_adc> &adc1_) : adc0(adc0_), adc1(adc1_) {};

    ADCBundle(const float (&adc_0_scales)[channels_within_mcp_adc],
                const float (&adc_0_offsets)[channels_within_mcp_adc],
                const float (&adc_1_scales)[channels_within_mcp_adc],
                const float (&adc_1_offsets)[channels_within_mcp_adc]) :
        adc0(ADC0_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_0_scales, adc_0_offsets),
        adc1(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_1_scales, adc_1_offsets)
    {}

    MCP_ADC <channels_within_mcp_adc> adc0;
    MCP_ADC <channels_within_mcp_adc> adc1;
};
using ADCSingletonInstance = etl::singleton<ADCBundle>; // Singleton for ADCs. Used to pass ADCs to other systems that need them, such as the TelemetrySystem.
#endif /* VCR_GLOBALS */