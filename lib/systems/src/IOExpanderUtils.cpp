#include "IOExpanderUtils.h"

bool IOExpanderUtils::getBit(uint16_t data, int port, int bit){
    if(!port){ //0=A
        return (data>>bit)&1;
    }
    return (data>>(7+bit))&1;
}