/******************************************************************************
 * @file    IOExpander.h
 * @brief   Source for the IOExpander
 ******************************************************************************/

 /******************************************************************************
 * Includes
 ******************************************************************************/
#include "IOExpander.h"
#include "SharedFirmwareTypes.h"

 /******************************************************************************
 * Public Method Definitions
 ******************************************************************************/
IOExpander::IOExpander(uint8_t addr) : _mcp(MCP23017(addr, Wire2)) {
    Wire2.begin();
    _mcp.init();
    
    constexpr uint8_t bit_mask = 0b1111111;
    _mcp.portMode(MCP23017Port::A, bit_mask);
    _mcp.portMode(MCP23017Port::B, bit_mask);

    // mcp.writeRegister(MCP23017Register::GPIO_A, 0x00);  // Reset port A 
    // mcp.writeRegister(MCP23017Register::GPIO_B, 0x00);  // Reset port B
    
    _mcp.writeRegister(MCP23017Register::GPPU_B, bit_mask);  // Internal pull-ups
    _mcp.writeRegister(MCP23017Register::GPPU_A, bit_mask);  // Internal pull-ups

    _mcp.writeRegister(MCP23017Register::IPOL_A, bit_mask);  // Polarity (inverted)
    _mcp.writeRegister(MCP23017Register::IPOL_B, bit_mask);  // Polarity (inverted)
}

uint16_t IOExpander::read() {
    return _mcp.read();
}

static bool getBit(uint16_t data, bool port, int bit) {
    constexpr size_t bits_in_byte = 8;
    if (!port) { // 0=A
        return (data >> bit) & 1;
    }
    return (data >> (bits_in_byte + bit)) & 1;
}