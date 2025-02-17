#ifndef __MCP23017INTERFACE_H__
#define __MCP23017INTERFACE_H__

#include <sys/types.h>

// 
namespace specifiedBit
{
    bool getBit(u_int16_t data, bool port, int bit);
}

#endif // __MCP23017INTERFACE_H__