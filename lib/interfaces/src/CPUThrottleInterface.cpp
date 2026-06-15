#include "CPUThrottleInterface.h"

volatile bool g_high_temp_alarm = false;
volatile bool g_low_temp_alarm = false;

void setup_cpu_throttle()
{
    // Set the CPU clock to 600 MHz
    set_arm_clock(DEFAULT_CLK_RATE);
    // Configure the internal temperature sensor
    InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &high_alarm_isr);
}

void low_alarm_isr()
{
    set_arm_clock(DEFAULT_CLK_RATE);
    g_low_temp_alarm = true;

    // set an alarm at high temperature
    InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &high_alarm_isr);
}

void high_alarm_isr()
{
    set_arm_clock(HIGH_TEMP_CLK_RATE);
    g_high_temp_alarm = true;

    // set an alarm at low temperature
    InternalTemperature.attachLowTempInterruptCelsius (LOW_TEMP, &low_alarm_isr);
}