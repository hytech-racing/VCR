#include <InternalTemperature.h>

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

#define LOW_TEMP 65
#define HIGH_TEMP 70
#define DEFAULT_CLK_RATE 600000000
#define HIGH_TEMP_CLK_RATE 150000000

void setup_cpu_throttle();

void low_alarm_isr();

void high_alarm_isr();
