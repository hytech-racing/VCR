#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <etl/singleton.h>
#include "MCP23017.h"

class IOExpander
{
public:
    IOExpander(uint8_t addr);

private:
    MCP23017 mcp;
    uint16_t data;
    
public:
    /* Get data from IOExpander.
    * Note this only updates the data read. To update any values to be used, please use getBit()
    */
    void read();
    /* Gets data from specified bit from specified port
    @param port 0=A; 1=B
    */
    bool getBit(bool port, int bit);

};

using IOExpanderInstance = etl::singleton<IOExpander>;

#endif /* IOEXPANDER_H */