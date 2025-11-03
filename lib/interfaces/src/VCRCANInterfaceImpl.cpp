#include "VCRCANInterfaceImpl.h"

#include "hytech.h"
#include <cstdint>

namespace VCRCANInterfaceImpl {

// global forwards
CANRXBufferType CAN1_rxBuffer; // can1 = aux can
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

    // Front Left TTPMS sensor
 // Front Left
    case LF_TTPMS_1_CANID:
        interfaces.lf_ttpms_interface.receivePressureAndVoltage(msg, millis, TTPMSWheelLocation::LF);
        break;
    case LF_TTPMS_2_CANID:
        interfaces.lf_ttpms_interface.receiveTemperatureCh1To4(msg, millis, TTPMSWheelLocation::LF);
        break;
    case LF_TTPMS_3_CANID:
        interfaces.lf_ttpms_interface.receiveTemperatureCh5To8(msg, millis, TTPMSWheelLocation::LF);
        break;
    case LF_TTPMS_4_CANID:
        interfaces.lf_ttpms_interface.receiveTemperatureCh9To12(msg, millis, TTPMSWheelLocation::LF);
        break;
    case LF_TTPMS_5_CANID:
        interfaces.lf_ttpms_interface.receiveTemperatureCh13To16(msg, millis, TTPMSWheelLocation::LF);
        break;
    case LF_TTPMS_6_CANID:
        interfaces.lf_ttpms_interface.receiveSensorData(msg, millis, TTPMSWheelLocation::LF);
        break;

    // Front Right
    case RF_TTPMS_1_CANID:
        interfaces.fr_ttpms_interface.receivePressureAndVoltage(msg, millis, TTPMSWheelLocation::RF);
        break;
    case RF_TTPMS_2_CANID:
        interfaces.fr_ttpms_interface.receiveTemperatureCh1To4(msg, millis, TTPMSWheelLocation::RF);
        break;
    case RF_TTPMS_3_CANID:
        interfaces.fr_ttpms_interface.receiveTemperatureCh5To8(msg, millis, TTPMSWheelLocation::RF);
        break;
    case RF_TTPMS_4_CANID:
        interfaces.fr_ttpms_interface.receiveTemperatureCh9To12(msg, millis, TTPMSWheelLocation::RF);
        break;
    case RF_TTPMS_5_CANID:
        interfaces.fr_ttpms_interface.receiveTemperatureCh13To16(msg, millis, TTPMSWheelLocation::RF);
        break;
    case RF_TTPMS_6_CANID:
        interfaces.fr_ttpms_interface.receiveSensorData(msg, millis, TTPMSWheelLocation::RF);
        break;

    // Rear Left
    case LR_TTPMS_1_CANID:
        interfaces.lr_ttpms_interface.receivePressureAndVoltage(msg, millis, TTPMSWheelLocation::LR);
        break;
    case LR_TTPMS_2_CANID:
        interfaces.lr_ttpms_interface.receiveTemperatureCh1To4(msg, millis, TTPMSWheelLocation::LR);
        break;
    case LR_TTPMS_3_CANID:
        interfaces.lr_ttpms_interface.receiveTemperatureCh5To8(msg, millis, TTPMSWheelLocation::LR);
        break;
    case LR_TTPMS_4_CANID:
        interfaces.lr_ttpms_interface.receiveTemperatureCh9To12(msg, millis, TTPMSWheelLocation::LR);
        break;
    case LR_TTPMS_5_CANID:
        interfaces.lr_ttpms_interface.receiveTemperatureCh13To16(msg, millis, TTPMSWheelLocation::LR);
        break;
    case LR_TTPMS_6_CANID:
        interfaces.lr_ttpms_interface.receiveSensorData(msg, millis, TTPMSWheelLocation::LR);
        break;

    // Rear Right
    case RR_TTPMS_1_CANID:
        interfaces.rr_ttpms_interface.receivePressureAndVoltage(msg, millis, TTPMSWheelLocation::RR);
        break;
    case RR_TTPMS_2_CANID:
        interfaces.rr_ttpms_interface.receiveTemperatureCh1To4(msg, millis, TTPMSWheelLocation::RR);
        break;
    case RR_TTPMS_3_CANID:
        interfaces.rr_ttpms_interface.receiveTemperatureCh5To8(msg, millis, TTPMSWheelLocation::RR);
        break;
    case RR_TTPMS_4_CANID:
        interfaces.rr_ttpms_interface.receiveTemperatureCh9To12(msg, millis, TTPMSWheelLocation::RR);
        break;
    case RR_TTPMS_5_CANID:
        interfaces.rr_ttpms_interface.receiveTemperatureCh13To16(msg, millis, TTPMSWheelLocation::RR);
        break;
    case RR_TTPMS_6_CANID:
        interfaces.rr_ttpms_interface.receiveSensorData(msg, millis, TTPMSWheelLocation::RR);
        break;

    default:
        break;
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
}
 // namespace VCRCANInterfaceImpl
