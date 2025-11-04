#ifndef ADCINTERFACE_H
#define ADCINTERFACE_H

#include "VCR_Globals.h"
#include "VCR_Constants.h"

#include "MCP_ADC.h"
#include "SharedFirmwareTypes.h"

#include "etl/singleton.h"

class ADCInterface
{
public:
  ADCInterface() : _adc0(
      ADC0_CS,
      MCP_ADC_DEFAULT_SPI_SDI,
      MCP_ADC_DEFAULT_SPI_SDO,
      MCP_ADC_DEFAULT_SPI_CLK,
      MCP_ADC_DEFAULT_SPI_SPEED,
      ([] {
        static float arr[channels_within_mcp_adc] = {}; 
        arr[GLV_SENSE_CHANNEL]        = GLV_SENSE_SCALE;
        arr[CURRENT_SENSE_CHANNEL]    = CURRENT_SENSE_SCALE;
        arr[REFERENCE_SENSE_CHANNEL]  = REFERENCE_SENSE_SCALE;
        arr[RL_LOADCELL_CHANNEL]      = RL_LOADCELL_SCALE;
        arr[RR_LOADCELL_CHANNEL]      = RR_LOADCELL_SCALE;
        arr[RL_SUS_POT_CHANNEL]       = RL_SUS_POT_SCALE;
        arr[RR_SUS_POT_CHANNEL]       = RR_SUS_POT_SCALE;
        return arr;
      }()),
      ([] {
        static float arr[channels_within_mcp_adc] = {};
        arr[GLV_SENSE_CHANNEL]        = GLV_SENSE_OFFSET;
        arr[CURRENT_SENSE_CHANNEL]    = CURRENT_SENSE_OFFSET;
        arr[REFERENCE_SENSE_CHANNEL]  = REFERENCE_SENSE_OFFSET;
        arr[RL_LOADCELL_CHANNEL]      = RL_LOADCELL_OFFSET;
        arr[RR_LOADCELL_CHANNEL]      = RR_LOADCELL_OFFSET;
        arr[RL_SUS_POT_CHANNEL]       = RL_SUS_POT_OFFSET;
        arr[RR_SUS_POT_CHANNEL]       = RR_SUS_POT_OFFSET;
        return arr;
      }())
    ),
    _adc1(
      ADC1_CS,
      MCP_ADC_DEFAULT_SPI_SDI,
      MCP_ADC_DEFAULT_SPI_SDO,
      MCP_ADC_DEFAULT_SPI_CLK,
      MCP_ADC_DEFAULT_SPI_SPEED,
      ([] {
          static float arr[channels_within_mcp_adc] = {};
          arr[THERMISTOR_0] = THERMISTOR_0_SCALE;
          arr[THERMISTOR_1] = THERMISTOR_1_SCALE;
          arr[THERMISTOR_2] = THERMISTOR_2_SCALE;
          arr[THERMISTOR_3] = THERMISTOR_3_SCALE;
          arr[THERMISTOR_4] = THERMISTOR_4_SCALE;
          arr[THERMISTOR_5] = THERMISTOR_5_SCALE;
          arr[THERMISTOR_6] = THERMISTOR_6_SCALE;
          arr[THERMISTOR_7] = THERMISTOR_7_SCALE;
          return arr;
      }()),
      ([] {
          static float arr[channels_within_mcp_adc] = {};
          arr[THERMISTOR_0] = THERMISTOR_0_OFFSET;
          arr[THERMISTOR_1] = THERMISTOR_1_OFFSET;
          arr[THERMISTOR_2] = THERMISTOR_2_OFFSET;
          arr[THERMISTOR_3] = THERMISTOR_3_OFFSET;
          arr[THERMISTOR_4] = THERMISTOR_4_OFFSET;
          arr[THERMISTOR_5] = THERMISTOR_5_OFFSET;
          arr[THERMISTOR_6] = THERMISTOR_6_OFFSET;
          arr[THERMISTOR_7] = THERMISTOR_7_OFFSET;
          return arr;
      }())
    ) {} 
        
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
  AnalogConversion_s read_glv();

  AnalogConversion_s read_current();

  AnalogConversion_s read_reference();

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

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>; // Singleton for ADCs. Used to pass ADCs to other systems that need them, such as the TelemetrySystem.



#endif
