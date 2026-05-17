#include "mcp2515.h"

#include <Arduino.h>



MCP2515 mcp2515(10);
struct can_frame frame;


void setup() {
    Serial.begin(115200);

    SPI.begin();
    
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
    //mcp2515.setNormalMode();

    mcp2515.setListenOnlyMode();
    SPI.begin();

}

void loop() {

    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        Serial.println("error_ok");
    }

    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_FAIL) {
        Serial.println("error_fail");
    }

    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_NOMSG) {
        Serial.println("error_fail");
    }
    
    
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {

    
        Serial.print("ID: ");
        Serial.print(frame.can_id, HEX);

        Serial.print(" DLC: ");
        Serial.print(frame.can_dlc);

        Serial.print(" Data: ");
        for (int i = 0; i < frame.can_dlc; i++) {
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
}    
