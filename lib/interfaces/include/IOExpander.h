/******************************************************************************
 * @file    IOExpander.h
 * @brief   Header for the IOExpander
 ******************************************************************************/
#ifndef IOEXPANDER_H
#define IOEXPANDER_H

/******************************************************************************
 * Includes
 ******************************************************************************/
#include <etl/singleton.h>
#include "MCP23017.h"

/******************************************************************************
 * Public Class Declarations
 ******************************************************************************/
/**
 * @class IOExpander
 * A single instance of this class encapsulates all the methods for reading data from the IOExpander
 */
class IOExpander {
    public:
        /**
         * Constructs an instance of the IOExpander
         * @param addr the I2C address of the IOExpander
         */
        IOExpander(uint8_t addr);
        
        /**
         * Gets data from IOExpander. To retrieve updated values, use IOExpanderUtils::getBit()
         * @return 16-bit value read from the IOExpander
         */
        uint16_t read();

        /**
         * Utility function to extract a specific bit from the 16-bit data read from IOExpander
         * @param data 16-bit data read from IOExpander
         * @param port false for Port A, true for Port B
         * @param bit bit position (0-7) within the specified port
         * @return boolean value of the specified bit
         */
        static bool getBit(uint16_t data, bool port, int bit);

    private:
        MCP23017 _mcp;

};

using IOExpanderInstance = etl::singleton<IOExpander>;

#endif /* IOEXPANDER_H */