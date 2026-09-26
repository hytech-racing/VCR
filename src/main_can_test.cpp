#include <Arduino.h>

#include "CANInterface.h"
#include "ht_can.h"

FlexCAN_T4<CAN2> MAIN_CAN;
const uint32_t CAN_BAUDRATE = 500000;


void setOperationalMode(uint8_t req_state) {
    SET_OPERATING_MODE_t set_operating_mode_msg;
    set_operating_mode_msg.requested_state = req_state;
    set_operating_mode_msg.target_node = 0x011;

    CAN_message_t msg;
    msg.id = Pack_SET_OPERATING_MODE_ht_can(&set_operating_mode_msg, msg.buf, &msg.len, (uint8_t *) &msg.flags.extended);

    MAIN_CAN.write(msg);
}

void on_recv(const CAN_message_t &msg)
{
    // Serial.print("MB: "); Serial.print(msg.mb);
    // Serial.print("  ID: 0x"); Serial.print(msg.id, HEX);
    // Serial.print("  EXT: "); Serial.print(msg.flags.extended);
    // Serial.print("  LEN: "); Serial.print(msg.len);
    // Serial.print(" DATA: ");
    // for ( uint8_t i = 0; i < 8; i++ ) {
    //   Serial.print(msg.buf[i]); Serial.print(" ");
    // }
    // Serial.print("  TS: "); Serial.println(msg.timestamp);

    switch (msg.id) {
        case RSS_BOOT_UP_CANID:
            RSS_BOOT_UP_t boot_msg;
            Unpack_RSS_BOOT_UP_ht_can(&boot_msg, &msg.buf[0], msg.len);
            Serial.println("RSS Boot Msg: ");
            Serial.print("  Data: "); Serial.println(boot_msg.rss_initialization, HEX);

            setOperationalMode(0x01);
        break;
        case RSS_STATUS_CANID:
            RSS_STATUS_t status_msg;
            Unpack_RSS_STATUS_ht_can(&status_msg, &msg.buf[0], msg.len);
            Serial.println("RSS Status Msg: ");
            Serial.print("  E-Stop pressed 1: "); Serial.println(status_msg.emergency_stop_pressed_1);
            Serial.print("  E-Stop pressed 2: "); Serial.println(status_msg.emergency_stop_pressed_2);
            Serial.print("  Button pressed: "); Serial.println(status_msg.button_k3_pressed);
            Serial.print("  Switch on: "); Serial.println(status_msg.switch_k2_pressed);
            Serial.print("  Radio Link quality: "); Serial.println(status_msg.radio_link_quality);
            Serial.print("  Correct mode selected: "); Serial.println(status_msg.correct_mode_selected);
            Serial.print("  Pre-alarm warning: "); Serial.println(status_msg.pre_alarm_warning);
        break;
        default:
            Serial.println("unrecognized msg");
        break;
    }
}

void setup()
{
    handle_CAN_setup(MAIN_CAN, CAN_BAUDRATE, &on_recv);
    
    // setOperationalMode(0x80);
}

void loop()
{
    delay(100);
}