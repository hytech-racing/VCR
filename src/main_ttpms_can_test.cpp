#include <Arduino.h>

#include "CANInterface.h"
#include "TTPMSSensorInterface.h"
#include "hytech.h"


FlexCAN_T4<CAN2> MAIN_CAN;

void on_recv(const CAN_message_t &msg)
{
    
    Serial.print("MB: "); Serial.print(msg.mb);
    Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    Serial.print("  LEN: "); Serial.print(msg.len);
    Serial.print(" DATA: ");
    Serial.print("  TS: "); Serial.println(msg.timestamp);
}
    

void setup()
{
    Serial.begin(115200);
    handle_CAN_setup(MAIN_CAN, 500000, &on_recv);
}

void loop()
{    
    delay(1000);

}

 // TTPMSSensorData_s high_level_data = {
    //     .serial_number = 123,
    //     .battery_voltage = 220,
    //     .pressure = 228,
    //     .gauge_pressure = 227,
    //     .infrared_temp = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16},
    //     .transmission_count = 22,
    //     .rssi = 33,
    //     .sensor_temperature = 44,
    //     .sensor_node_id = 55
    // };

    // LF_TTPMS_1_t packed = {
    // .LF_TTPMS_SN      = high_level_data.serial_number,
    // .LF_TTPMS_BAT_V   = high_level_data.battery_voltage,
    // .LF_TTPMS_P       = high_level_data.pressure,
    // .LF_TTPMS_P_GAUGE = high_level_data.gauge_pressure
    // };


    // test_msg.id = 0x11;
    // test_msg.len = 1;
    // test_msg.buf[0] = 0x45;
    //     // memmove(test_msg.buf, &packed, sizeof(packed));

    // MAIN_CAN.write(test_msg);

        // Serial.print("SN: ");       Serial.println(unpacked.LF_TTPMS_SN);
    // Serial.print("Bat V: ");    Serial.println(unpacked.LF_TTPMS_BAT_V);
    // Serial.print("P: ");        Serial.println(unpacked.LF_TTPMS_P);
    // Serial.print("P Gauge: ");  Serial.println(unpacked.LF_TTPMS_P_GAUGE);

        // LF_TTPMS_1_t unpacked;
    // Unpack_LF_TTPMS_1_hytech(&unpacked, msg.buf, msg.len);