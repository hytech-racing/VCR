#include "IOExpanderUtils.h"
#include <cstddef>

bool IOExpanderUtils::getBit(uint16_t data, bool port, int bit) {
    constexpr size_t bits_in_byte = 8;
    if (!port) { // 0=A
        return (data >> bit) & 1;
    }
    return (data >> (bits_in_byte + bit)) & 1;
}