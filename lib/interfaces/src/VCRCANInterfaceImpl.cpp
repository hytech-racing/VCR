#include "VCRCANInterfaceImpl.h"

#include "hytech.h"
#include <cstdint>
#include <iostream>

namespace VCRCANInterfaceImpl {

// global forwards
CANRXBufferType CAN1_rxBuffer;
CANRXBufferType inverter_can_rx_buffer;
CANRXBufferType telem_can_rx_buffer;

CANTXBufferType CAN1_txBuffer;
CANTXBufferType inverter_can_tx_buffer;
CANTXBufferType telem_can_tx_buffer;



void on_can1_receive(const CAN_message_t &msg) {
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN1_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

void on_inverter_can_receive(const CAN_message_t &msg) {
    TELEM_CAN.write(msg); // send immediately onto the telem CAN line
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    inverter_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
}

void on_telem_can_receive(const CAN_message_t &msg) {
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    telem_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
}

void vcr_CAN_recv(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis) {
    switch (msg.id) {

    case PEDALS_SYSTEM_DATA_CANID: 
    {
        interfaces.vcf_interface.receive_pedals_message(msg, millis);
        break;
    }
    case FRONT_SUSPENSION_CANID:
    {
        interfaces.vcf_interface.receive_front_suspension_message(msg, millis);
        break;
    }
    case DASH_INPUT_CANID:
    {
        interfaces.vcf_interface.receive_dashboard_message(msg, millis);
        break;
    }
    case ACU_OK_CANID:
    {
        interfaces.acu_interface.receive_acu_ok_message(msg, millis);
        break;
    }
    case DRIVEBRAIN_TORQUE_LIM_INPUT_CANID: 
    {
        interfaces.db_interface.receive_drivebrain_torque_lim_command(msg, millis);
        break;
    }
    case DRIVEBRAIN_SPEED_SET_INPUT_CANID: {
        interfaces.db_interface.receive_drivebrain_speed_command(msg, millis);
        break;
    }

    // Front Left Inverter
    {
        case INV1_STATUS_CANID: {
            interfaces.fl_inverter_interface.receive_INV_STATUS(msg, millis); 
            break;
        }
        case INV1_TEMPS_CANID: {
            interfaces.fl_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV1_DYNAMICS_CANID: {
            interfaces.fl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV1_POWER_CANID: {
            interfaces.fl_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV1_FEEDBACK_CANID: {
            interfaces.fl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
    }

    // Front right inverter
    {
        case INV2_STATUS_CANID: {
            interfaces.fr_inverter_interface.receive_INV_STATUS(msg, millis); 
            break;
        }
        case INV2_TEMPS_CANID: {
            interfaces.fr_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV2_DYNAMICS_CANID: {
            interfaces.fr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV2_POWER_CANID: {
            interfaces.fr_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV2_FEEDBACK_CANID: {
            interfaces.fr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
    }

    // Rear left inverter
    {
        case INV3_STATUS_CANID: {
            interfaces.rl_inverter_interface.receive_INV_STATUS(msg, millis); 
            break;
        }
        case INV3_TEMPS_CANID: {
            interfaces.rl_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV3_DYNAMICS_CANID: {
            interfaces.rl_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV3_POWER_CANID: {
            interfaces.rl_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV3_FEEDBACK_CANID: {
            interfaces.rl_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
    }

    // Rear right inverter
    {
        case INV4_STATUS_CANID: {
            interfaces.rr_inverter_interface.receive_INV_STATUS(msg, millis); 
            break;
        }
        case INV4_TEMPS_CANID: {
            interfaces.rr_inverter_interface.receive_INV_TEMPS(msg, millis);
            break;
        }
        case INV4_DYNAMICS_CANID: {
            interfaces.rr_inverter_interface.receive_INV_DYNAMICS(msg, millis);
            break;
        }
        case INV4_POWER_CANID: {
            interfaces.rr_inverter_interface.receive_INV_POWER(msg, millis);
            break;
        }
        case INV4_FEEDBACK_CANID: {
            interfaces.rr_inverter_interface.receive_INV_FEEDBACK(msg, millis);
            break;
        }
    }
    

    
    // fl wheel
    {
        case LF_TTPMS_1_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_pressure_and_voltage(msg, millis, TTPMS_Wheel_Location::LF);
            break;
        }
        case LF_TTPMS_2_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_temp_lf_ch1_ch4(msg, millis);
            break;
        }
        case LF_TTPMS_3_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_temp_lf_ch5_ch8(msg, millis);
            break;
        }
        case LF_TTPMS_4_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_temp_lf_ch9_ch12(msg, millis);
            break;
        }
        case LF_TTPMS_5_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_temp_lf_ch13_ch16(msg, millis);
            break;
        }
        case LF_TTPMS_6_CANID: {
            interfaces.fl_ttpms_interface.receive_TTPMS_sensor_sensor_data(msg, millis, TTPMS_Wheel_Location::LF);
            break;
        }
    }

   // RF wheel
{
    case RF_TTPMS_1_CANID: {
        Serial.println("Receiving pressure and voltage");

        interfaces.fr_ttpms_interface.receive_TTPMS_sensor_pressure_and_voltage(msg, millis, TTPMS_Wheel_Location::RF);
        break;
    }
    case RF_TTPMS_2_CANID: {
        interfaces.fr_ttpms_interface.receive_TTPMS_sensor_temp_rf_ch1_ch4(msg, millis);
        break;
    }
    case RF_TTPMS_3_CANID: {
        interfaces.fr_ttpms_interface.receive_TTPMS_sensor_temp_rf_ch5_ch8(msg, millis);
        break;
    }
    case RF_TTPMS_4_CANID: {
        interfaces.fr_ttpms_interface.receive_TTPMS_sensor_temp_rf_ch9_ch12(msg, millis);
        break;
    }
    case RF_TTPMS_5_CANID: {
        interfaces.fr_ttpms_interface.receive_TTPMS_sensor_temp_rf_ch13_ch16(msg, millis);
        break;
    }
        case RF_TTPMS_6_CANID: {
            interfaces.fr_ttpms_interface.receive_TTPMS_sensor_sensor_data(msg, millis, TTPMS_Wheel_Location::RF);
            break;
    }
}

    // LR wheel
    {
        case LR_TTPMS_1_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_pressure_and_voltage(msg, millis, TTPMS_Wheel_Location::LR);
            break;
        }
        case LR_TTPMS_2_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_temp_lr_ch1_ch4(msg, millis);
            break;
        }
        case LR_TTPMS_3_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_temp_lr_ch5_ch8(msg, millis);
            break;
        }
        case LR_TTPMS_4_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_temp_lr_ch9_ch12(msg, millis);
            break;
        }
        case LR_TTPMS_5_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_temp_lr_ch13_ch16(msg, millis);
            break;
        }
        case LR_TTPMS_6_CANID: {
            interfaces.rl_ttpms_interface.receive_TTPMS_sensor_sensor_data(msg, millis, TTPMS_Wheel_Location::LR);
            break;
        }
    }

    // RR wheel
    {
        case RR_TTPMS_1_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_pressure_and_voltage(msg, millis, TTPMS_Wheel_Location::RR);
            break;
        }
        case RR_TTPMS_2_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_temp_rr_ch1_ch4(msg, millis);
            break;
        }
        case RR_TTPMS_3_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_temp_rr_ch5_ch8(msg, millis);
            break;
        }
        case RR_TTPMS_4_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_temp_rr_ch9_ch12(msg, millis);
            break;
        }
        case RR_TTPMS_5_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_temp_rr_ch13_ch16(msg, millis);
            break;
        }
        case RR_TTPMS_6_CANID: {
            interfaces.rr_ttpms_interface.receive_TTPMS_sensor_sensor_data(msg, millis, TTPMS_Wheel_Location::RR);
            break;
        }
    }

    default: {
        break;
    }
}
}

void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface) {
    CAN_message_t msg;
    while (buffer.available()) {
        CAN_message_t msg;
        uint8_t buf[sizeof(CAN_message_t)];
        buffer.pop_front(buf, sizeof(CAN_message_t));
        memmove(&msg, buf, sizeof(msg));
        can_interface->write(msg);
    }
}
} // namespace VCRCANInterfaceImpl
