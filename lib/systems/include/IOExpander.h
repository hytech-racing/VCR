#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <etl/singleton.h>
#include "MCP23017.h"

class IOExpander
{
public:
    IOExpander();

private:
    MCP23017 mcp;
    uint16_t data;
    
public:
    /* Get/update watchdog state */
    void read();
    bool getBit(bool port, int bit);

};

using IOExpanderInstance = etl::singleton<IOExpander>;

#endif /* IOEXPANDER_H */