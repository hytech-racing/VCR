
#include "SystemTimeInterface.h"

// mock system time interface

unsigned long millis_time, micros_time;

namespace sys_time {
unsigned long hal_millis() { return millis_time; }
unsigned long hal_micros() { return micros_time; }

void set_millis(unsigned long m)
{
    millis_time = m;
}

void set_micros(unsigned long m)
{
    micros_time = m;
}

} // namespace sys_time
