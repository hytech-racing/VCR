#include "MCP23017Interface.h"

#include "SharedFirmwareTypes.h"

namespace specifiedBit
{
    bool getBit(uint16_t data, bool port, int bit){
        if(!port){ //0=A
            return (data>>bit)&1;
        }
        return (data>>(7+bit))&1;
    }
}