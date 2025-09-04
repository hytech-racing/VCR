#include "TTPMSSensorInterface.h"
#include "FlexCAN_T4.h"
#include "hytech.h"

void TTPMSSensorInterface::receive_TTPMS_sensor_pressure_and_voltage(const CAN_message_t &msg, unsigned long curr_millis, TTPMS_Wheel_Location wheel_location) {
    Serial.println("Receiving pressure and voltage");
    switch (wheel_location) {
        case TTPMS_Wheel_Location::LF:
            UNPACK_TTPMS_1(LF);
            break;
        case TTPMS_Wheel_Location::RF:
            UNPACK_TTPMS_1(RF);
            break;
        case TTPMS_Wheel_Location::LR:
            UNPACK_TTPMS_1(LR);
            break;
        case TTPMS_Wheel_Location::RR:
            UNPACK_TTPMS_1(RR);
            break;
    }
    // if (1) {
    //     RF_TTPMS_1_t rf_ttpms_1;
    //     Unpack_RF_TTPMS_1_hytech(&rf_ttpms_1, msg.buf, msg.len);
    //     Serial.print("Serial Number: "); Serial.println(rf_ttpms_1.RF_TTPMS_SN);
    // }
}


void TTPMSSensorInterface::receive_TTPMS_sensor_sensor_data(const CAN_message_t &msg, unsigned long curr_millis, TTPMS_Wheel_Location wheel_location) {
    switch (wheel_location) {
        case TTPMS_Wheel_Location::LF:
            UNPACK_TTPMS_6(LF);
            break;
        case TTPMS_Wheel_Location::RF:
            UNPACK_TTPMS_6(RF);
            break;
        case TTPMS_Wheel_Location::LR:
            UNPACK_TTPMS_6(LR);
            break;
        case TTPMS_Wheel_Location::RR:
            UNPACK_TTPMS_6(RR);
            break;
    }
}

void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LF, 2, 1, 2, 3, 4);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LF, 3, 5, 6, 7, 8);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LF, 4, 9, 10, 11, 12);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LF, 5, 13, 14, 15, 16);
}

// RF wheel
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RF, 2, 1, 2, 3, 4);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RF, 3, 5, 6, 7, 8);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RF, 4, 9, 10, 11, 12);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RF, 5, 13, 14, 15, 16);
}

// LR wheel
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LR, 2, 1, 2, 3, 4);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LR, 3, 5, 6, 7, 8);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LR, 4, 9, 10, 11, 12);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_lr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(LR, 5, 13, 14, 15, 16);
}

// RR wheel
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RR, 2, 1, 2, 3, 4);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RR, 3, 5, 6, 7, 8);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RR, 4, 9, 10, 11, 12);
}
void TTPMSSensorInterface::receive_TTPMS_sensor_temp_rr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis) {
    UNPACK_AND_POPULATE_TEMP(RR, 5, 13, 14, 15, 16);
}

TTPMSSensorData_s TTPMSSensorInterface::get_latest_sensor_data() const {
    return latest_sensors_data;
}