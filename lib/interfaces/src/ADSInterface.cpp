#include "ADSInterface.h"
// Data is 24-bit 
void ADSInterface::init_uart() {
    Serial1.begin(9600);
    attachInterrupt(digitalPinToInterrupt(),"func()?", FALLING); // ? gpio2 is tied to gnd
}

void ADSInterface::init() {
    Serial1.write(10101010); // synch word
    Serial1.write(00000110); // reset
    
    // writing register configs
    Serial1.write(10101010);
    Serial1.write(0100000000000000); // Register 0 MUX[3:0] GAIN[2:0] PGA_BYPASS
    
    Serial1.write(10101010);
    Serial1.write(0100001000100110); // Register 1 DR[2:0] MODE CM VREF[1:0] TS
    
    Serial1.write(10101010);
    Serial1.write(0100010000100000); // Register 2 DRDY DCNT CRC[1:0] BCS IDAC[2:0]
    
    Serial1.write(10101010);
    Serial1.write(0100011000000000); // Register 3 I1MUX[2:0] I2MUX[2:0] RESERVED AUTO
    
    Serial1.write(10101010);
    Serial1.write(0100100000000000); // Register 4 RESERVED, GPIO STUFF



}

void ADSInterface::read_current_data() {
    // Read register 2 for configuration bit of DRDY
    Serial.write(0010100);
    // Initiate RDATA command to read data when available based on state of DRDY pin
    // DRDY pin becomes low when conversion is complete and high again when data is latched
    Serial.write(0001000);
    // Periodically read the DRDY bit in the configuration register. 
    // If set, the DRDY bit indicates that a new conversion result is ready 
    // for retrieval. We can subsequently issue an RDATA command 
    // to retrieve the data. The rate at which we host 
    // polls the ADS122U04 for new data must be at least as 
    // fast as the data rate in continuous conversion 
    // mode to prevent the host from missing a conversion result.


}

void ADSInterface::power_up() {
    delay(0.6);
    Serial1.write(010101010);
}

void ADSInterface::power_down() {
    Serial1.write(10101010);
    Serial1.write(00000010); //  shutdown

}