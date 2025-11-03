#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include "VCR_Globals.h"

#include "MCP_ADC.h"
#include "SharedFirmwareTypes.h"

#include "etl/singleton.h"

class ADCInterface
{
public:
  

  ADCInterface(const float (&adc_0_scales)[channels_within_mcp_adc],
               const float (&adc_0_offsets)[channels_within_mcp_adc],
               const float (&adc_1_scales)[channels_within_mcp_adc],
               const float (&adc_1_offsets)[channels_within_mcp_adc]) :
        _adc0(ADC0_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_0_scales, adc_0_offsets),
        _adc1(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_1_scales, adc_1_offsets) {}

  /**
  * Samples from ADC0
  */
  void tick_adc0();
  
  /**
  * Samples from ADC1
  */
  void tick_adc1();

  /* -------------------- ADC0 -------------------- */

  /**
   * @return The reading of the 24V sensor analog channel
  */
  AnalogConversion_s read_twentyfour_volt_sensor();

  AnalogConversion_s read_current_sensor();

  AnalogConversion_s read_current_reference();

  AnalogConversion_s read_rl_loadcell();

  AnalogConversion_s read_rr_loadcell();

  AnalogConversion_s read_rl_sus_pot();

  AnalogConversion_s read_rr_sus_pot();

  /* -------------------- ADC1 -------------------- */
  
  AnalogConversion_s read_thermistor_0();

  AnalogConversion_s read_thermistor_1();
  
  AnalogConversion_s read_thermistor_2(); 

  AnalogConversion_s read_thermistor_3();

  AnalogConversion_s read_thermistor_4();
  
  AnalogConversion_s read_thermistor_5();

  AnalogConversion_s read_thermistor_6();

  AnalogConversion_s read_thermistor_7();

private:
  MCP_ADC<channels_within_mcp_adc> _adc0;
  MCP_ADC<channels_within_mcp_adc> _adc1;

  /**
  * @pre ADC interface is constructed and initialized
  * @post 
  * @param channel The ADC0 channel to read from 
  * @return The ADC channel's output packet
  */
  AnalogConversionPacket_s<channels_within_mcp_adc> _read_adc0_channel(int channel);

  /**
  * @pre ADC interface is constructed and initialized
  * @post 
  * @param channel The ADC1 channel to read from 
  * @return The ADC channel's output packet
  */
  AnalogConversionPacket_s<channels_within_mcp_adc> _read_adc1_channel(int channel);

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif
