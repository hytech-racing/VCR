#include "LLSysInterface.h"

#include <Arduino.h>

namespace ll_sys
{
    unsigned long ll_millis()
    {
        return millis();
    }

    unsigned long ll_micros()
    {
        return micros();
    }
}