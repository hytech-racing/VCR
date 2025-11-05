#include "ADCInterface.h"

void ADCInterface::init() {

  std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_scales = {};
  std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_offsets = {};
  std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_scales = {};
  std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_offsets = {};

  adc0_scales.at(_adc_parameters.channels.glv_sense_channel)          = _adc_parameters.scales.glv_sense_scale; 
  adc0_scales.at(_adc_parameters.channels.current_sense_channel)      = _adc_parameters.scales.current_sense_scale;
  adc0_scales.at(_adc_parameters.channels.reference_sense_channel)    = _adc_parameters.scales.reference_sense_scale;
  adc0_scales.at(_adc_parameters.channels.rl_loadcell_channel)        = _adc_parameters.scales.rl_loadcell_scale;
  adc0_scales.at(_adc_parameters.channels.rr_loadcell_channel)        = _adc_parameters.scales.rr_loadcell_scale;
  adc0_scales.at(_adc_parameters.channels.rl_suspot_channel)          = _adc_parameters.scales.rl_suspot_scale;
  adc0_scales.at(_adc_parameters.channels.rr_suspot_channel)          = _adc_parameters.scales.rr_suspot_scale;

  adc0_offsets.at(_adc_parameters.channels.glv_sense_channel)         = _adc_parameters.offsets.glv_sense_offset; 
  adc0_offsets.at(_adc_parameters.channels.current_sense_channel)     = _adc_parameters.offsets.current_sense_offset;
  adc0_offsets.at(_adc_parameters.channels.reference_sense_channel)   = _adc_parameters.offsets.reference_sense_offset;
  adc0_offsets.at(_adc_parameters.channels.rl_loadcell_channel)       = _adc_parameters.offsets.rl_loadcell_offset;
  adc0_offsets.at(_adc_parameters.channels.rr_loadcell_channel)       = _adc_parameters.offsets.rr_loadcell_offset;
  adc0_offsets.at(_adc_parameters.channels.rl_suspot_channel)         = _adc_parameters.offsets.rl_suspot_offset;
  adc0_offsets.at(_adc_parameters.channels.rr_suspot_channel)         = _adc_parameters.offsets.rr_suspot_offset;

  adc1_scales.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.scales.thermistor0_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor1_channel)        = _adc_parameters.scales.thermistor1_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor2_channel)        = _adc_parameters.scales.thermistor2_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor3_channel)        = _adc_parameters.scales.thermistor3_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor4_channel)        = _adc_parameters.scales.thermistor4_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor5_channel)        = _adc_parameters.scales.thermistor5_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor6_channel)        = _adc_parameters.scales.thermistor6_scale;
  adc1_scales.at(_adc_parameters.channels.thermistor7_channel)        = _adc_parameters.scales.thermistor7_scale;

  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor0_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor1_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor2_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor3_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor4_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor5_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor6_offset;
  adc1_offsets.at(_adc_parameters.channels.thermistor0_channel)        = _adc_parameters.offsets.thermistor7_offset;

  _adc0.emplace(
    _adc_parameters.pinouts.adc0_spi_cs_pin, 
    MCP_ADC_DEFAULT_SPI_SDI, 
    MCP_ADC_DEFAULT_SPI_SDO, 
    MCP_ADC_DEFAULT_SPI_CLK,
    MCP_ADC_DEFAULT_SPI_SPEED,
    adc0_scales.data(),
    adc0_offsets.data()
  );

  _adc1.emplace(
    _adc_parameters.pinouts.adc1_spi_cs_pin, 
    MCP_ADC_DEFAULT_SPI_SDI, 
    MCP_ADC_DEFAULT_SPI_SDO, 
    MCP_ADC_DEFAULT_SPI_CLK,
    MCP_ADC_DEFAULT_SPI_SPEED,
    adc1_scales.data(),
    adc1_offsets.data()
  );
}

void ADCInterface::tick_adc0() { _adc0->tick(); }
void ADCInterface::tick_adc1() { _adc1->tick(); }

const ADCInterfaceParams_s& ADCInterface::get_adc_params() const {
    return _adc_parameters;
}

AnalogConversion_s ADCInterface::read_glv() {
  return _adc0->data.conversions.at(_adc_parameters.channels.glv_sense_channel);
}

AnalogConversion_s ADCInterface::read_bspd_current() {
  return _adc0->data.conversions.at(_adc_parameters.channels.current_sense_channel);
}

AnalogConversion_s ADCInterface::read_bspd_reference_current() {
  return _adc0->data.conversions.at(_adc_parameters.channels.reference_sense_channel);
}

AnalogConversion_s ADCInterface::read_rl_loadcell() {
  return _adc0->data.conversions.at(_adc_parameters.channels.rl_loadcell_channel);
}

AnalogConversion_s ADCInterface::read_rr_loadcell() {
  return _adc0->data.conversions.at(_adc_parameters.channels.rr_loadcell_channel);
}

AnalogConversion_s ADCInterface::read_rl_sus_pot() {
  return _adc0->data.conversions.at(_adc_parameters.channels.rl_suspot_channel);
}

AnalogConversion_s ADCInterface::read_rr_sus_pot() {
  return _adc0->data.conversions.at(_adc_parameters.channels.rr_suspot_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_0() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor0_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_1() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor1_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_2() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor2_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_3() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor3_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_4() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor4_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_5() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor5_channel);  
}

AnalogConversion_s ADCInterface::read_thermistor_6() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor6_channel);
}

AnalogConversion_s ADCInterface::read_thermistor_7() {
  return _adc1->data.conversions.at(_adc_parameters.channels.thermistor7_channel);
}
