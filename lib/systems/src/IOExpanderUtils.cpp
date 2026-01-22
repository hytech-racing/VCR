#include "IOExpanderUtils.h"
#include <cstddef>

// data is the list of bits recieved. port is either a or b (7 ports mirrored on both sides so they are distinguished by a/b)
// bit is the value read there (0 or 1)
bool IOExpanderUtils::getBit(uint16_t data, bool port, int bit) {
    constexpr size_t bits_in_byte = 8;
    if (!port) { // 0=A
        return (data >> bit) & 1;
    }
    return (data >> (bits_in_byte + bit)) & 1;
}