#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include "MCP_ADC.h"
#include "etl/singleton.h"

#include <array>
#include <optional>

namespace adc_default_parameters {
  constexpr const unsigned int channels_within_mcp_adc = 8;
}

struct ADCPinout_s {
  int adc0_spi_cs_pin;
  int adc1_spi_cs_pin;
};

struct ADCChannels_s {
  /* ADC 0 */
  int glv_sense_channel;
  int current_sense_channel;
  int reference_sense_channel;
  int rl_loadcell_channel;
  int rr_loadcell_channel;
  int rl_suspot_channel;
  int rr_suspot_channel;

  /* ADC 1 */
  int thermistor0_channel;
  int thermistor1_channel;
  int thermistor2_channel;
  int thermistor3_channel;
  int thermistor4_channel;
  int thermistor5_channel;
  int thermistor6_channel;
  int thermistor7_channel;
};

struct ADCScales_s {
  float glv_sense_scale;
  float current_sense_scale;
  float reference_sense_scale;
  float rl_loadcell_scale;
  float rr_loadcell_scale;
  float rl_suspot_scale;
  float rr_suspot_scale;

  float thermistor0_scale;
  float thermistor1_scale;
  float thermistor2_scale;
  float thermistor3_scale;
  float thermistor4_scale;
  float thermistor5_scale;
  float thermistor6_scale;
  float thermistor7_scale;
};

struct ADCOffsets_s {
  float glv_sense_offset;
  float current_sense_offset;
  float reference_sense_offset;
  float rl_loadcell_offset;
  float rr_loadcell_offset;
  float rl_suspot_offset;
  float rr_suspot_offset;

  float thermistor0_offset;
  float thermistor1_offset;
  float thermistor2_offset;
  float thermistor3_offset;
  float thermistor4_offset;
  float thermistor5_offset;
  float thermistor6_offset;
  float thermistor7_offset;
};

struct ADCInterfaceParams_s {
  ADCPinout_s pinouts;
  ADCChannels_s channels;
  ADCScales_s scales;
  ADCOffsets_s offsets;
};

class ADCInterface
{
public:
  ADCInterface(ADCPinout_s pinouts, ADCChannels_s channels, ADCScales_s scales, ADCOffsets_s offsets) : _adc_parameters { 
    pinouts,
    channels,
    scales,
    offsets } {};

  /**
   * @pre constructor called and ADC parameters created
   * @post _adc0 and _adc1 initialized with correct scales and offsets
  */
  void init();
        
  /**
  * Samples from ADC0
  */
  void tick_adc0();
  
  /**
  * Samples from ADC1
  */
  void tick_adc1();

  const ADCInterfaceParams_s& get_adc_params() const;

  /* -------------------- ADC0 -------------------- */

  /**
   * @return The reading of the 24V sensor analog channel
  */
  AnalogConversion_s read_glv();
  
  /**
   * @return The reading of the BSPD current analog channel
  */
  AnalogConversion_s read_bspd_current();

  /**
   * @return The reading of the BSPD reference current analog channel
  */
  AnalogConversion_s read_bspd_reference_current();
  
  /**
   * @return The reading of the rear left load cell analog channel
  */
  AnalogConversion_s read_rl_loadcell();
  
  /**
   * @return The reading of the rear right load cell analog channel
  */
  AnalogConversion_s read_rr_loadcell();
  
  /**
   * @return The reading of the rear left suspension potentiometer analog channel
  */
  AnalogConversion_s read_rl_sus_pot();
  
  /**
   * @return The reading of the rear right suspension potentiometer analog channel
  */ 
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
  ADCInterfaceParams_s _adc_parameters = {};

  std::optional<MCP_ADC<adc_default_parameters::channels_within_mcp_adc>> _adc0;
  std::optional<MCP_ADC<adc_default_parameters::channels_within_mcp_adc>> _adc1;

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>; // Singleton for ADCs. Used to pass ADCs to other systems that need them, such as the TelemetrySystem.

#endif
