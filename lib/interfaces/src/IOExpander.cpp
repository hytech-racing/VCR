#include "IOExpander.h"
#include "SharedFirmwareTypes.h"

/* Returns data from IOExpander */
IOExpander::IOExpander(uint8_t addr) : mcp(MCP23017(addr))
{
    mcp.init();
    mcp.portMode(MCP23017Port::A, 0b01111111);
    mcp.portMode(MCP23017Port::B, 0b01111111);

    mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

    // GPIO_B reflects the same logic as the input pins state
    mcp.writeRegister(MCP23017Register::IPOL_A, 0x00);
    mcp.writeRegister(MCP23017Register::IPOL_B, 0x00);
}

void IOExpander::read() 
{
    data = mcp.read();
}

bool IOExpander::getBit(bool port, int bit){
    if(!port){ //0=A
        return (data>>bit)&1;
    }
    return (data>>(7+bit))&1;
}