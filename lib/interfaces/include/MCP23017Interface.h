#ifndef __MCP23017INTERFACE_H__
#define __MCP23017INTERFACE_H__


#include "SharedFirmwareTypes.h"

namespace specifiedBit
{
    bool getBit(uint16_t data, bool port, int bit);
}

#endif // __MCP23017INTERFACE_H__