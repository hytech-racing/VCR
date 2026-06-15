#include "CPUThrottleInterface.h"

volatile bool g_high_temp_alarm = false;
volatile bool g_low_temp_alarm = false;

void setup_cpu_throttle()
{
    // Set the CPU clock to 600 MHz
    set_arm_clock(600000000);
    // Configure the internal temperature sensor
    InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &high_alarm_ISR);
}

void low_alarm_ISR()
{
    set_arm_clock(600000000);
    g_low_temp_alarm = true;

    // set an alarm at high temperature
    InternalTemperature.attachHighTempInterruptCelsius (HIGH_TEMP, &high_alarm_ISR);
}

void high_alarm_ISR()
{
    set_arm_clock(150000000);
    g_high_temp_alarm = true;

    // set an alarm at low temperature
    InternalTemperature.attachLowTempInterruptCelsius (LOW_TEMP, &low_alarm_ISR);
}