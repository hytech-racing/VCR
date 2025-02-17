#include "MCP23017Interface.h"
#include <sys/types.h>

namespace specifiedBit
{
    bool get_bit(u_int16_t data, bool port, int bit){
        if(!port){ //0=A
            return (data>>bit)&1;
        }
        return (data>>7+bit)&1;
    }
}