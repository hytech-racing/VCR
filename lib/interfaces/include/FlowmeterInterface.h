#ifndef FLOWMETERINTERFACE_H
#define FLOWMETERINTERFACE_H

#include "core_pins.h"
#include "etl/singleton.h"

class FlowmeterInterface {
public:
  FlowmeterInterface(int flowmeter_pin) {
    pinMode(flowmeter_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(flowmeter_pin), count_pulse, RISING);
  };

  static void count_pulse() { etl::singleton<FlowmeterInterface>::instance()._pulse_count++; };

  float get_flow() { 
    float reading = 0.0183 * _pulse_count * 5; // NOLINT
    _pulse_count = 0;
    return reading;
  };

private:
  unsigned long _pulse_count = 0;
};

using FlowmeterInterfaceInstance = etl::singleton<FlowmeterInterface>;

#endif
