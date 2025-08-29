#ifndef TTMPSSENSORINTERFACE_H
#define TTMPSSENSORINTERFACE_H

#include "FlexCAN_T4.h"
#include "shared_types.h"

class TTPMSSensorInterface {
    public:
        void receive_TTPMS_sensor_temp(const CAN_message_t &msg, unsigned long curr_millis, uint8_t start_channel);
        void receive_TTPMS_sensor_pressure_and_voltage(const CAN_message_t &msg, unsigned long curr_millis);
        void receive_TTPMS_sensor_sensor_data(const CAN_message_t &msg, unsigned long curr_millis);

        TTPMSSensorData_s get_latest_sensor_data() const;
    private:
        TTPMSSensors_s latest_sensors_data;
};

#endif 