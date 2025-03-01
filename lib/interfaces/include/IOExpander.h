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
    
public:
    /* Gets data from IOExpander.
    * To retrieve updated values, use getBit()
    */
    uint16_t read();
    /* read() only reads the data! This function is only used to get specified bit from previously read dataframe
    @param port 0=A; 1=B
    */
    // bool getBit(bool port, int bit);

};

using IOExpanderInstance = etl::singleton<IOExpander>;

#endif /* IOEXPANDER_H */