#include "TTPMSInterface.h"
#include "hytech.h"
#include <array>

// ---------------- Pressure & Battery ----------------
void TTPMSInterface::receivePressureAndVoltage(
    const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel) 
{
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_1_t data;
            Unpack_LF_TTPMS_1_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.battery_voltage = data.LF_TTPMS_BAT_V;
            _latest_sensor_data.serial_number   = data.LF_TTPMS_SN;
            _latest_sensor_data.pressure        = HYTECH_LF_TTPMS_P_ro_fromS(data.LF_TTPMS_P_ro);
            _latest_sensor_data.gauge_pressure  = data.LF_TTPMS_P_GAUGE;
            break;
        }
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_1_t data;
            Unpack_RF_TTPMS_1_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.battery_voltage = data.RF_TTPMS_BAT_V;
            _latest_sensor_data.serial_number   = data.RF_TTPMS_SN;
            _latest_sensor_data.pressure        = HYTECH_RF_TTPMS_P_ro_fromS(data.RF_TTPMS_P_ro);
            _latest_sensor_data.gauge_pressure  = data.RF_TTPMS_P_GAUGE;
            break;
        }
        case TTPMSWheelLocation::LR: {
            LR_TTPMS_1_t data;
            Unpack_LR_TTPMS_1_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.battery_voltage = data.LR_TTPMS_BAT_V;
            _latest_sensor_data.serial_number   = data.LR_TTPMS_SN;
            _latest_sensor_data.pressure        = HYTECH_LR_TTPMS_P_ro_fromS(data.LR_TTPMS_P_ro);
            _latest_sensor_data.gauge_pressure  = data.LR_TTPMS_P_GAUGE;
            break;
        }
        case TTPMSWheelLocation::RR: {
            RR_TTPMS_1_t data;
            Unpack_RR_TTPMS_1_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.battery_voltage = data.RR_TTPMS_BAT_V;
            _latest_sensor_data.serial_number   = data.RR_TTPMS_SN;
            _latest_sensor_data.pressure        = HYTECH_RR_TTPMS_P_ro_fromS(data.RR_TTPMS_P_ro);
            _latest_sensor_data.gauge_pressure  = data.RR_TTPMS_P_GAUGE;
            break;
        }
    }
}

void TTPMSInterface::receiveSensorData(
    const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel)
{
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_6_t data;
            Unpack_LF_TTPMS_6_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.rssi               = data.LF_TTPMS_RSSI;
            _latest_sensor_data.sensor_node_id     = data.LF_TTPMS_NODE_ID;
            _latest_sensor_data.transmission_count = data.LF_TTPMS_TC;
            _latest_sensor_data.sensor_temperature = HYTECH_LF_TTPMS_T_ro_fromS(data.LF_TTPMS_T_ro);
            break;
        }
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_6_t data;
            Unpack_RF_TTPMS_6_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.rssi               = data.RF_TTPMS_RSSI;
            _latest_sensor_data.sensor_node_id     = data.RF_TTPMS_NODE_ID;
            _latest_sensor_data.transmission_count = data.RF_TTPMS_TC;
            _latest_sensor_data.sensor_temperature = HYTECH_RF_TTPMS_T_ro_fromS(data.RF_TTPMS_T_ro);
            break;
        }
        case TTPMSWheelLocation::LR: {
            LR_TTPMS_6_t data;
            Unpack_LR_TTPMS_6_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.rssi               = data.LR_TTPMS_RSSI;
            _latest_sensor_data.sensor_node_id     = data.LR_TTPMS_NODE_ID;
            _latest_sensor_data.transmission_count = data.LR_TTPMS_TC;
            _latest_sensor_data.sensor_temperature = HYTECH_LR_TTPMS_T_ro_fromS(data.LR_TTPMS_T_ro);
            break;
        }
        case TTPMSWheelLocation::RR: {
            RR_TTPMS_6_t data;
            Unpack_RR_TTPMS_6_hytech(&data, msg.buf, msg.len);
            _latest_sensor_data.rssi               = data.RR_TTPMS_RSSI;
            _latest_sensor_data.sensor_node_id     = data.RR_TTPMS_NODE_ID;
            _latest_sensor_data.transmission_count = data.RR_TTPMS_TC;
            _latest_sensor_data.sensor_temperature = HYTECH_RR_TTPMS_T_ro_fromS(data.RR_TTPMS_T_ro);
            break;
        }
    }
}

// ---------------- Infrared Temperature ----------------
void TTPMSInterface::_updateInfraredTemp(double baseIndex, const std::array<double,4>& values) {
    for (size_t i = 0; i < 4; ++i) {
        _latest_sensor_data.infrared_temp[baseIndex + i] = values[i];
    }
}

void TTPMSInterface::receiveTemperatureCh1To4(const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel) {
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_2_t data;
            Unpack_LF_TTPMS_2_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(0, std::array<double,4>{
                HYTECH_LF_TTPMS_T1_ro_fromS(data.LF_TTPMS_T1_ro), 
                HYTECH_LF_TTPMS_T2_ro_fromS(data.LF_TTPMS_T2_ro), 
                HYTECH_LF_TTPMS_T3_ro_fromS(data.LF_TTPMS_T3_ro), 
                HYTECH_LF_TTPMS_T4_ro_fromS(data.LF_TTPMS_T4_ro)
            });
            break;
        }
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_2_t data;
            Unpack_RF_TTPMS_2_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(0, std::array<double,4>{
                HYTECH_RF_TTPMS_T1_ro_fromS(data.RF_TTPMS_T1_ro), 
                HYTECH_RF_TTPMS_T2_ro_fromS(data.RF_TTPMS_T2_ro), 
                HYTECH_RF_TTPMS_T3_ro_fromS(data.RF_TTPMS_T3_ro), 
                HYTECH_RF_TTPMS_T4_ro_fromS(data.RF_TTPMS_T4_ro)
            });
            break;
        }
        case TTPMSWheelLocation::LR: {
            LR_TTPMS_2_t data;
            Unpack_LR_TTPMS_2_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(0, std::array<double,4>{
                HYTECH_LR_TTPMS_T1_ro_fromS(data.LR_TTPMS_T1_ro), 
                HYTECH_LR_TTPMS_T2_ro_fromS(data.LR_TTPMS_T2_ro), 
                HYTECH_LR_TTPMS_T3_ro_fromS(data.LR_TTPMS_T3_ro), 
                HYTECH_LR_TTPMS_T4_ro_fromS(data.LR_TTPMS_T4_ro)
            });
            break;
        }
        case TTPMSWheelLocation::RR: {
            RR_TTPMS_2_t data;
            Unpack_RR_TTPMS_2_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(0, std::array<double,4>{
                HYTECH_RR_TTPMS_T1_ro_fromS(data.RR_TTPMS_T1_ro), 
                HYTECH_RR_TTPMS_T2_ro_fromS(data.RR_TTPMS_T2_ro), 
                HYTECH_RR_TTPMS_T3_ro_fromS(data.RR_TTPMS_T3_ro), 
                HYTECH_RR_TTPMS_T4_ro_fromS(data.RR_TTPMS_T4_ro)
            });
            break;
        }
    }
}

void TTPMSInterface::receiveTemperatureCh5To8(const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel) {
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_3_t data;
            Unpack_LF_TTPMS_3_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(4, std::array<double,4>{
                HYTECH_LF_TTPMS_T5_ro_fromS(data.LF_TTPMS_T5_ro), 
                HYTECH_LF_TTPMS_T6_ro_fromS(data.LF_TTPMS_T6_ro), 
                HYTECH_LF_TTPMS_T7_ro_fromS(data.LF_TTPMS_T7_ro), 
                HYTECH_LF_TTPMS_T8_ro_fromS(data.LF_TTPMS_T8_ro)
            });
            break;
        }
        
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_3_t data;
            Unpack_RF_TTPMS_3_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(4, std::array<double,4>{
                HYTECH_RF_TTPMS_T5_ro_fromS(data.RF_TTPMS_T5_ro), 
                HYTECH_RF_TTPMS_T6_ro_fromS(data.RF_TTPMS_T6_ro), 
                HYTECH_RF_TTPMS_T7_ro_fromS(data.RF_TTPMS_T7_ro), 
                HYTECH_RF_TTPMS_T8_ro_fromS(data.RF_TTPMS_T8_ro)
            });
            break;
        }

        case TTPMSWheelLocation::LR: {
            LR_TTPMS_3_t data;
            Unpack_LR_TTPMS_3_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(4, std::array<double,4>{
                HYTECH_LR_TTPMS_T5_ro_fromS(data.LR_TTPMS_T5_ro), 
                HYTECH_LR_TTPMS_T6_ro_fromS(data.LR_TTPMS_T6_ro), 
                HYTECH_LR_TTPMS_T7_ro_fromS(data.LR_TTPMS_T7_ro), 
                HYTECH_LR_TTPMS_T8_ro_fromS(data.LR_TTPMS_T8_ro)
            });
            break;
        }

        case TTPMSWheelLocation::RR: {
            RR_TTPMS_3_t data;
            Unpack_RR_TTPMS_3_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(4, std::array<double,4>{
                HYTECH_RR_TTPMS_T5_ro_fromS(data.RR_TTPMS_T5_ro), 
                HYTECH_RR_TTPMS_T6_ro_fromS(data.RR_TTPMS_T6_ro), 
                HYTECH_RR_TTPMS_T7_ro_fromS(data.RR_TTPMS_T7_ro), 
                HYTECH_RR_TTPMS_T8_ro_fromS(data.RR_TTPMS_T8_ro)
            });
            break;
        }
    }
}

void TTPMSInterface::receiveTemperatureCh9To12(const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel) {
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_4_t data;
            Unpack_LF_TTPMS_4_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(8, std::array<double,4>{
                HYTECH_LF_TTPMS_T9_ro_fromS(data.LF_TTPMS_T9_ro), 
                HYTECH_LF_TTPMS_T10_ro_fromS(data.LF_TTPMS_T10_ro), 
                HYTECH_LF_TTPMS_T11_ro_fromS(data.LF_TTPMS_T11_ro), 
                HYTECH_LF_TTPMS_T12_ro_fromS(data.LF_TTPMS_T12_ro)
            });
            break;
        }
        
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_4_t data;
            Unpack_RF_TTPMS_4_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(8, std::array<double,4>{
                HYTECH_RF_TTPMS_T9_ro_fromS(data.RF_TTPMS_T9_ro), 
                HYTECH_RF_TTPMS_T10_ro_fromS(data.RF_TTPMS_T10_ro), 
                HYTECH_RF_TTPMS_T11_ro_fromS(data.RF_TTPMS_T11_ro), 
                HYTECH_RF_TTPMS_T12_ro_fromS(data.RF_TTPMS_T12_ro)
            });
            break;
        }

        case TTPMSWheelLocation::LR: {
            LR_TTPMS_4_t data;
            Unpack_LR_TTPMS_4_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(8, std::array<double,4>{
                HYTECH_LR_TTPMS_T9_ro_fromS(data.LR_TTPMS_T9_ro), 
                HYTECH_LR_TTPMS_T10_ro_fromS(data.LR_TTPMS_T10_ro), 
                HYTECH_LR_TTPMS_T11_ro_fromS(data.LR_TTPMS_T11_ro), 
                HYTECH_LR_TTPMS_T12_ro_fromS(data.LR_TTPMS_T12_ro)
            });
            break;
        }

        case TTPMSWheelLocation::RR: {
            RR_TTPMS_4_t data;
            Unpack_RR_TTPMS_4_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(8, std::array<double,4>{
                HYTECH_RR_TTPMS_T9_ro_fromS(data.RR_TTPMS_T9_ro), 
                HYTECH_RR_TTPMS_T10_ro_fromS(data.RR_TTPMS_T10_ro), 
                HYTECH_RR_TTPMS_T11_ro_fromS(data.RR_TTPMS_T11_ro), 
                HYTECH_RR_TTPMS_T12_ro_fromS(data.RR_TTPMS_T12_ro)
            });
            break;
        }
    }
}

void TTPMSInterface::receiveTemperatureCh13To16(const CAN_message_t &msg, unsigned long millis, TTPMSWheelLocation wheel) {
    switch (wheel) {
        case TTPMSWheelLocation::LF: {
            LF_TTPMS_5_t data;
            Unpack_LF_TTPMS_5_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(12, std::array<double,4>{
                HYTECH_LF_TTPMS_T13_ro_fromS(data.LF_TTPMS_T13_ro), 
                HYTECH_LF_TTPMS_T14_ro_fromS(data.LF_TTPMS_T14_ro), 
                HYTECH_LF_TTPMS_T15_ro_fromS(data.LF_TTPMS_T15_ro), 
                HYTECH_LF_TTPMS_T16_ro_fromS(data.LF_TTPMS_T16_ro)
            });
            break;
        }
        
        case TTPMSWheelLocation::RF: {
            RF_TTPMS_5_t data;
            Unpack_RF_TTPMS_5_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(12, std::array<double,4>{
                HYTECH_RF_TTPMS_T13_ro_fromS(data.RF_TTPMS_T13_ro), 
                HYTECH_RF_TTPMS_T14_ro_fromS(data.RF_TTPMS_T14_ro), 
                HYTECH_RF_TTPMS_T15_ro_fromS(data.RF_TTPMS_T15_ro), 
                HYTECH_RF_TTPMS_T16_ro_fromS(data.RF_TTPMS_T16_ro)
            });
            break;
        }

        case TTPMSWheelLocation::LR: {
            LR_TTPMS_5_t data;
            Unpack_LR_TTPMS_5_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(12, std::array<double,4>{
                HYTECH_LR_TTPMS_T13_ro_fromS(data.LR_TTPMS_T13_ro), 
                HYTECH_LR_TTPMS_T14_ro_fromS(data.LR_TTPMS_T14_ro), 
                HYTECH_LR_TTPMS_T15_ro_fromS(data.LR_TTPMS_T15_ro), 
                HYTECH_LR_TTPMS_T16_ro_fromS(data.LR_TTPMS_T16_ro)
            
            });
            break;
        }

        case TTPMSWheelLocation::RR: {
            RR_TTPMS_5_t data;
            Unpack_RR_TTPMS_5_hytech(&data, msg.buf, msg.len);
            _updateInfraredTemp(12, std::array<double,4>{
                
                HYTECH_RR_TTPMS_T13_ro_fromS(data.RR_TTPMS_T13_ro), 
                HYTECH_RR_TTPMS_T14_ro_fromS(data.RR_TTPMS_T14_ro), 
                HYTECH_RR_TTPMS_T15_ro_fromS(data.RR_TTPMS_T15_ro), 
                HYTECH_RR_TTPMS_T16_ro_fromS(data.RR_TTPMS_T16_ro)
            });
            break;
        }
    }
}

// ---------------- Get/Set ----------------
TTPMSSensorData_s TTPMSInterface::getLatestSensorData() const {
    return _latest_sensor_data;
}

