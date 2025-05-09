#include "IOExpander.h"
#include "SharedFirmwareTypes.h"

/* Returns data from IOExpander */
IOExpander::IOExpander(uint8_t addr) : mcp(MCP23017(addr, Wire2))
{
    Wire2.begin();
    mcp.init();
    
    constexpr uint8_t bit_mask = 0b1111111;
    mcp.portMode(MCP23017Port::A, bit_mask);
    mcp.portMode(MCP23017Port::B, bit_mask);

    // mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  //Reset port A 
    // mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  //Reset port B
    
    mcp.writeRegister(MCP23017Register::GPPU_B, bit_mask);  //Internal pull-ups
    mcp.writeRegister(MCP23017Register::GPPU_A, bit_mask);  //Internal pull-ups

    mcp.writeRegister(MCP23017Register::IPOL_A, bit_mask);  //Polarity (inverted)
    mcp.writeRegister(MCP23017Register::IPOL_B, bit_mask);  //Polarity (inverted)
}

uint16_t IOExpander::read() 
{
    return mcp.read();
}