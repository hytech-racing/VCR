#include "ADCInterface.h"

void ADCInterface::tick_adc0() { _adc0.tick(); }
void ADCInterface::tick_adc1() { _adc1.tick(); }

AnalogConversion_s ADCInterface::read_glv() {
  return _adc0.data.conversions[GLV_SENSE_CHANNEL];
}

AnalogConversion_s ADCInterface::read_bspd_current() {
  return _adc0.data.conversions[CURRENT_SENSE_CHANNEL];
}

AnalogConversion_s ADCInterface::read_bspd_reference_current() {
  return _adc0.data.conversions[REFERENCE_SENSE_CHANNEL];
}

AnalogConversion_s ADCInterface::read_rl_loadcell() {
  return _adc0.data.conversions[RL_LOADCELL_CHANNEL];
}

AnalogConversion_s ADCInterface::read_rr_loadcell() {
  return _adc0.data.conversions[RR_LOADCELL_CHANNEL];
}

AnalogConversion_s ADCInterface::read_rl_sus_pot() {
  return _adc0.data.conversions[RL_SUS_POT_CHANNEL];
}

AnalogConversion_s ADCInterface::read_rr_sus_pot() {
  return _adc0.data.conversions[RR_SUS_POT_CHANNEL];
}

AnalogConversion_s ADCInterface::read_thermistor_0() {
  return _adc1.data.conversions[THERMISTOR_0];
}

AnalogConversion_s ADCInterface::read_thermistor_1() {
  return _adc1.data.conversions[THERMISTOR_1];
}

AnalogConversion_s ADCInterface::read_thermistor_2() {
  return _adc1.data.conversions[THERMISTOR_2];
}

AnalogConversion_s ADCInterface::read_thermistor_3() {
  return _adc1.data.conversions[THERMISTOR_3];
}

AnalogConversion_s ADCInterface::read_thermistor_4() {
  return _adc1.data.conversions[THERMISTOR_4];
}

AnalogConversion_s ADCInterface::read_thermistor_5() {
  return _adc1.data.conversions[THERMISTOR_5];  
}

AnalogConversion_s ADCInterface::read_thermistor_6() {
  return _adc1.data.conversions[THERMISTOR_6];
}

AnalogConversion_s ADCInterface::read_thermistor_7() {
  return _adc1.data.conversions[THERMISTOR_7];
}
