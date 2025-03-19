#include "IOExpander.h"
#include "SharedFirmwareTypes.h"

/* Returns data from IOExpander */
IOExpander::IOExpander(uint8_t addr) : mcp(MCP23017(addr))
{
    mcp.init();
    constexpr uint8_t seven_bit_mask = 0b01111111;
    mcp.portMode(MCP23017Port::A, seven_bit_mask);
    mcp.portMode(MCP23017Port::B, seven_bit_mask);

    mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B

    // GPIO_B reflects the same logic as the input pins state
    mcp.writeRegister(MCP23017Register::IPOL_A, 0x00);
    mcp.writeRegister(MCP23017Register::IPOL_B, 0x00);
}

uint16_t IOExpander::read() 
{
    return mcp.read();
}