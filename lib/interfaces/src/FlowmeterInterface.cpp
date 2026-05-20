#include "FlowmeterInterface.h"

FlowmeterInterface::FlowmeterInterface(
    const size_t flowmeter_pin
) : 
    _pin(flowmeter_pin) 
{
    pinMode(_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(_pin), count_pulse, RISING);
};

float FlowmeterInterface::get_flow_gpm() 
{ 
    float reading = 0.0183 * _pulse_count * 5; // NOLINT
    _pulse_count = 0;
    return reading;
};