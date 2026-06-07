/*
 * ads112_4ch_test.cpp
 *
 * Tests all 4 channels single-ended vs AVSS.
 * gain=1, PGA bypass, AVDD=5V ref, 20SPS, single-shot.
 *
 * Wiring:
 *   Teensy Serial8 TX (pin 35) -> 47R -> ADS112U04 RX
 *   Teensy Serial8 RX (pin 34) -> 47R -> ADS112U04 TX
 */

#include <Arduino.h>

#define UART_PORT   Serial8
#define BAUD_RATE   115200
#define BYTE_GAP_US 1

static void clearRx()
{
    delay(5);
    while (UART_PORT.available()) UART_PORT.read();
}

static void sendCmd(uint8_t cmd)
{
    UART_PORT.write(0x55);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
    UART_PORT.write(cmd);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
}

static void writeReg(uint8_t reg, uint8_t val)
{
    uint8_t cmd = 0x40 | ((reg & 0x07) << 1);
    UART_PORT.write(0x55);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
    UART_PORT.write(cmd);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
    UART_PORT.write(val);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
}

static uint8_t readReg(uint8_t reg)
{
    uint8_t cmd = 0x20 | ((reg & 0x07) << 1);
    clearRx();
    UART_PORT.write(0x55);
    UART_PORT.flush();
    delayMicroseconds(BYTE_GAP_US);
    UART_PORT.write(cmd);
    UART_PORT.flush();

    uint32_t t = millis();
    while (!UART_PORT.available())
    {
        if (millis() - t > 200) return 0xFF;
    }
    delayMicroseconds(10);
    uint8_t val = 0xFF;
    while (UART_PORT.available()) val = (uint8_t)UART_PORT.read();
    return val;
}

// Read one single-ended channel. Returns raw count.
// reg0_val encodes the MUX for that channel.
static int16_t readChannel(uint8_t reg0_val)
{
    // Update MUX for this channel
    writeReg(0x00, reg0_val);

    clearRx();

    // Fire single-shot conversion
    sendCmd(0x08);  // START
    delayMicroseconds(50000);      // 20SPS = ~50ms, 70ms to be safe

    // Request result
    clearRx();
    sendCmd(0x10);  // RDATA

    uint32_t t = millis();
    while (UART_PORT.available() < 2)
    {
        if (millis() - t > 500)
        {
            Serial.println("  TIMEOUT");
            clearRx();
            return 0;
        }
    }

    delayMicroseconds(1000);
    while (UART_PORT.available() > 2) UART_PORT.read();

    uint8_t lsb = (uint8_t)UART_PORT.read();
    uint8_t msb = (uint8_t)UART_PORT.read();

    return (int16_t)((uint16_t)(msb << 8) | lsb);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    Serial.println("ADS112U04 4-channel test");
    Serial.println("=========================");
    Serial.println("AINx vs AVSS | gain=1 | PGA bypass | AVDD=5V ref | 20SPS");
    Serial.println();

    UART_PORT.begin(BAUD_RATE);
    delay(200);
    clearRx();

    // Reset
    Serial.println("Resetting...");
    sendCmd(0x06);
    delay(50);
    clearRx();

    // Verify reset state
    Serial.println("Register state after reset (expect all 0x00):");
    bool reset_ok = true;
    for (uint8_t i = 0; i <= 4; i++)
    {
        uint8_t v = readReg(i);
        Serial.print("  Reg"); Serial.print(i);
        Serial.print(" = 0x"); Serial.print(v, HEX);
        if      (v == 0x00) Serial.println("  PASS");
        else if (v == 0xFF) { Serial.println("  TIMEOUT"); reset_ok = false; }
        else                { Serial.println("  UNEXPECTED"); reset_ok = false; }
    }

    if (!reset_ok)
    {
        Serial.println("\nERROR: Reset failed. Check wiring and ground connection.");
        while (1) {}
    }

    // Write shared registers (Reg0 MUX will be updated per channel in loop)
    Serial.println("\nWriting shared registers...");
    writeReg(0x00, 0x81);  // AIN0 vs AVSS to start, gain=1, PGA bypass
    writeReg(0x01, 0x04);  // 20SPS, normal, single-shot, AVDD ref
    writeReg(0x02, 0x00);  // all off
    writeReg(0x03, 0x00);  // manual read mode, no IDACs
    writeReg(0x04, 0x48);  // GPIO2/DRDY as DRDY output

    // Verify writes
    Serial.println("Register readback:");
    uint8_t expected[5] = {0x81, 0x04, 0x00, 0x00, 0x48};
    bool write_ok = true;
    for (uint8_t i = 0; i <= 4; i++)
    {
        uint8_t v = readReg(i);
        Serial.print("  Reg"); Serial.print(i);
        Serial.print(": expected 0x"); Serial.print(expected[i], HEX);
        Serial.print("  got 0x"); Serial.print(v, HEX);
        if (v == expected[i]) Serial.println("  PASS");
        else { Serial.println("  FAIL"); write_ok = false; }
    }

    if (!write_ok)
    {
        Serial.println("\nERROR: Register write failed.");
        while (1) {}
    }

    clearRx();

    Serial.println("\nStarting reads...");
    Serial.println("CH0 (V)  | CH1 (V)  | CH2 (V)  | CH3 (V)");
    Serial.println("---------|----------|----------|--------");
}

void loop()
{
    // Reg0 MUX values for each channel vs AVSS, gain=1, PGA bypass
    // MUX=1000 (AIN0) = 0x81
    // MUX=1001 (AIN1) = 0x91
    // MUX=1010 (AIN2) = 0xA1
    // MUX=1011 (AIN3) = 0xB1
    const uint8_t mux[4] = {0x81, 0x91, 0xA1, 0xB1};

    float voltages[4];
    for (int ch = 0; ch < 4; ch++)
    {
        int16_t raw = readChannel(mux[ch]);
        voltages[ch] = ((float)raw / 32768.0f) * 5.0f;
    }

    for (int ch = 0; ch < 4; ch++)
    {
        Serial.print(voltages[ch], 4);
        Serial.print(" V");
        if (ch < 3) Serial.print("  |  ");
    }
    Serial.println();
}