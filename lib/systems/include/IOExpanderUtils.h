#ifndef IO_EXPANDER_UTILS_H
#define IO_EXPANDER_UTILS_H

/* Standard int library */
#include <stdint.h>

// bool IOExpanderUtils::getBit(uint16_t data, int port, int bit);

namespace IOExpanderUtils
{
    bool getBit(uint16_t data, int port, int bit);
}

#endif