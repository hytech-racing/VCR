#ifndef TTMPSSENSORINTERFACE_H
#define TTMPSSENSORINTERFACE_H

#include "FlexCAN_T4.h"
#include "shared_types.h"
#include <unordered_map>
#include <tuple>

enum TTPMS_Wheel_Location {
    LF,
    RF,
    LR,
    RR
};

enum TTPMS_Sensor_Channel {
    CH1_CH4=0,
    CH5_CH8=4,
    CH9_CH12=8,
    CH13_CH16=12
};



#define UNPACK_TTPMS_1(WHEEL) \
    WHEEL##_TTPMS_1_t unpacked_sensor_data_##WHEEL; \
    Unpack_##WHEEL##_TTPMS_1_hytech(&unpacked_sensor_data_##WHEEL, msg.buf, msg.len); \
    latest_sensors_data.battery_voltage = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_BAT_V; \
    latest_sensors_data.serial_number  = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_SN; \
    latest_sensors_data.pressure       = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_P; \
    latest_sensors_data.gauge_pressure = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_P_GAUGE;

#define UNPACK_TTPMS_6(WHEEL) \
    WHEEL##_TTPMS_6_t unpacked_sensor_data_##WHEEL; \
    Unpack_##WHEEL##_TTPMS_6_hytech(&unpacked_sensor_data_##WHEEL, msg.buf, msg.len); \
    latest_sensors_data.rssi                = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_RSSI; \
    latest_sensors_data.sensor_node_id      = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_NODE_ID; \
    latest_sensors_data.transmission_count  = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_TC; \
    latest_sensors_data.sensor_temperature  = unpacked_sensor_data_##WHEEL.WHEEL##_TTPMS_T_ro;

#define UNPACK_AND_POPULATE_TEMP(WHEEL, MSGNUM, OFFSET1, OFFSET2, OFFSET3, OFFSET4) \
    WHEEL##_TTPMS_##MSGNUM##_t unpacked_sensor_data; \
    Unpack_##WHEEL##_TTPMS_##MSGNUM##_hytech(&unpacked_sensor_data, msg.buf, msg.len); \
    latest_sensors_data.infrared_temp[OFFSET1-1] = unpacked_sensor_data.WHEEL##_TTPMS_T##OFFSET1##_ro; \
    latest_sensors_data.infrared_temp[OFFSET1] = unpacked_sensor_data.WHEEL##_TTPMS_T##OFFSET2##_ro; \
    latest_sensors_data.infrared_temp[OFFSET1+1] = unpacked_sensor_data.WHEEL##_TTPMS_T##OFFSET3##_ro; \
    latest_sensors_data.infrared_temp[OFFSET1+2] = unpacked_sensor_data.WHEEL##_TTPMS_T##OFFSET4##_ro;

class TTPMSSensorInterface {

    public:
        void receive_TTPMS_sensor_pressure_and_voltage(const CAN_message_t &msg, unsigned long curr_milli, TTPMS_Wheel_Location wheel_location);
        void receive_TTPMS_sensor_sensor_data(const CAN_message_t &msg, unsigned long curr_millis, TTPMS_Wheel_Location wheel_location);

        // LF wheel
        void receive_TTPMS_sensor_temp_lf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // RF wheel
        void receive_TTPMS_sensor_temp_rf_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rf_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rf_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rf_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // LR wheel
        void receive_TTPMS_sensor_temp_lr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_lr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        // RR wheel
        void receive_TTPMS_sensor_temp_rr_ch1_ch4(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rr_ch5_ch8(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rr_ch9_ch12(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_temp_rr_ch13_ch16(const CAN_message_t &msg, unsigned long curr_millis);

        TTPMSSensorData_s get_latest_sensor_data() const;

        void set_latest_sensor_data(const TTPMSSensorData_s &data);
    private:
        
        TTPMSSensorData_s latest_sensors_data;
};

#endif 