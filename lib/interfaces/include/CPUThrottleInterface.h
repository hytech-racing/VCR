#include <InternalTemperature.h>

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

#define LOW_TEMP 65
#define HIGH_TEMP 70

void setup_cpu_throttle();

void low_alarm_ISR();

void high_alarm_ISR();
