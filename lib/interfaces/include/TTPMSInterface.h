#ifndef TTPMSINTERFACE_H
#define TTPMSINTERFACE_H

#include "FlexCAN_T4.h"
#include "shared_types.h"
#include <unordered_map>
#include <tuple>
#include <array>
#include "hytech.h"

enum TTPMSWheelLocation {
    LF,
    RF,
    LR,
    RR
};

struct TTPMSSensorData_s {
    uint16_t battery_voltage;
    uint16_t serial_number;
    uint16_t pressure;
    uint16_t gauge_pressure;

    uint16_t rssi;
    uint16_t sensor_node_id;
    uint16_t transmission_count;
    uint16_t sensor_temperature;

    std::array<uint16_t, 16> infrared_temp{}; // T1-T16
};

class TTPMSInterface {
public:
    TTPMSInterface() = default;

    // Pressure and battery
    void receivePressureAndVoltage(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);

    // Sensor data (RSSI, node ID, TC, temperature)
    void receiveSensorData(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);

    // Infrared temperature channels
    void receiveTemperatureCh1To4(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);
    void receiveTemperatureCh5To8(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);
    void receiveTemperatureCh9To12(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);
    void receiveTemperatureCh13To16(const CAN_message_t &msg, unsigned long curr_millis, TTPMSWheelLocation wheel);

    TTPMSSensorData_s getLatestSensorData() const;
  

private:
    TTPMSSensorData_s _latest_sensor_data;

    void _updateInfraredTemp(uint16_t baseIndex, const std::array<uint16_t,4>& values);
};


#endif 