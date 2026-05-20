#ifndef FLOWMETERINTERFACE_H
#define FLOWMETERINTERFACE_H

#include <Arduino.h>
#include "etl/singleton.h"

class FlowmeterInterface {
public:
    FlowmeterInterface(const size_t flowmeter_pin);

    static void count_pulse() { etl::singleton<FlowmeterInterface>::instance()._pulse_count++; };

    float get_flow_gpm();

private:
    unsigned long _pulse_count = 0;
    size_t _pin;
};

using FlowmeterInterfaceInstance = etl::singleton<FlowmeterInterface>;

#endif // FLOWMETERINTERFACE_H
