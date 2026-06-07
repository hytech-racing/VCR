/*
 * ads112_ain0_test.cpp
 *
 * AIN0 vs AVSS, gain=1, PGA bypass, AVDD=5V ref, 20SPS, SINGLE-SHOT mode.
 *
 * Wiring:
 *   Teensy Serial8 TX (pin 35) -> 47R -> ADS112U04 RX
 *   Teensy Serial8 RX (pin 34) -> 47R -> ADS112U04 TX
 */

#include <Arduino.h>

#define UART_PORT  Serial3
#define BAUD_RATE  115200
#define BYTE_GAP_US 500

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
    delayMicroseconds(500);
    // drain to last byte in case of extras
    uint8_t val = 0xFF;
    while (UART_PORT.available()) val = (uint8_t)UART_PORT.read();
    return val;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    delay(500);

    Serial.println("ADS112U04 AIN0 single-shot test");
    Serial.println("=================================");

    UART_PORT.begin(BAUD_RATE);
    delay(200);
    clearRx();

    // Reset
    Serial.println("Resetting...");
    sendCmd(0x06);
    delay(50);
    clearRx();

    // Read registers after reset - all should be 0x00
    Serial.println("\nRegister state after reset (expect all 0x00):");
    for (uint8_t i = 0; i <= 4; i++)
    {
        uint8_t v = readReg(i);
        Serial.print("  Reg"); Serial.print(i);
        Serial.print(" = 0x"); Serial.print(v, HEX);
        Serial.print(" ("); Serial.print(v, BIN); Serial.print(")");
        if      (v == 0x00) Serial.println("  PASS");
        else if (v == 0xFF) Serial.println("  TIMEOUT");
        else                Serial.println("  UNEXPECTED");
    }

    // Write all 5 registers
    Serial.println("\nWriting registers...");
    writeReg(0x00, 0x81);  // AIN0 vs AVSS, gain=1, PGA bypass
    writeReg(0x01, 0x04);  // 20SPS, normal, single-shot, AVDD ref
    writeReg(0x02, 0x00);  // all off
    writeReg(0x03, 0x00);  // manual read mode, no IDACs
    writeReg(0x04, 0x48);  // GPIO2/DRDY as DRDY output

    delay(10);

    // Read back all registers to confirm writes
    Serial.println("\nRegister readback after write:");
    uint8_t expected[5] = {0x81, 0x04, 0x00, 0x00, 0x48};
    for (uint8_t i = 0; i <= 4; i++)
    {
        uint8_t v = readReg(i);
        Serial.print("  Reg"); Serial.print(i);
        Serial.print(": expected 0x"); Serial.print(expected[i], HEX);
        Serial.print("  got 0x"); Serial.print(v, HEX);
        Serial.println(v == expected[i] ? "  PASS" : "  FAIL");
    }

    clearRx();

    Serial.println("\nStarting reads...");
    Serial.println("bytes | raw    | voltage (V)");
    Serial.println("------|--------|------------");
}

void loop()
{
    clearRx();

    // Fire single-shot conversion
    sendCmd(0x08);  // START
    delay(70);      // 20SPS = ~50ms, 70ms to be safe

    // Request result
    clearRx();
    sendCmd(0x10);  // RDATA

    uint32_t t = millis();
    while (UART_PORT.available() < 2)
    {
        if (millis() - t > 500)
        {
            Serial.println("TIMEOUT");
            clearRx();
            return;
        }
    }

    delayMicroseconds(1000);
    int avail = UART_PORT.available();
    while (UART_PORT.available() > 2) UART_PORT.read();

    uint8_t lsb = (uint8_t)UART_PORT.read();
    uint8_t msb = (uint8_t)UART_PORT.read();

    int16_t raw = (int16_t)((uint16_t)(msb << 8) | lsb);
    float   vin = ((float)raw / 32768.0f) * 5.0f;

    Serial.print(avail);
    Serial.print("     | ");
    Serial.print(raw);
    Serial.print("  |  ");
    Serial.print(vin, 4);
    Serial.println(" V");
}