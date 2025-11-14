#ifndef FLOWMETERINTERFACE_H
#define FLOWMETERINTERFACE_H

#include "core_pins.h"
#include "etl/singleton.h"

class FlowmeterInterface {
public:
  FlowmeterInterface(const int flowmeter_pin) : _pin(flowmeter_pin) {
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), count_pulse, RISING);
  };

  static void count_pulse() { etl::singleton<FlowmeterInterface>::instance()._pulse_count++; };

  float get_flow() { 
    float reading = 0.0183 * _pulse_count * 5; // NOLINT
    _pulse_count = 0;
    return reading;
  };

private:
  unsigned long _pulse_count = 0;
  int _pin;
};

using FlowmeterInterfaceInstance = etl::singleton<FlowmeterInterface>;

#endif
