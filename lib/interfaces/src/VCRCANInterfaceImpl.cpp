#include "VCRCANInterfaceImpl.h"

#include "hytech.h"
#include <cstdint>
// // global forwards
CANRXBufferType CAN1_rxBuffer;
CANRXBufferType inverter_can_rx_buffer;
CANRXBufferType telem_can_rx_buffer;

CANTXBufferType CAN1_txBuffer;
CANTXBufferType inverter_can_tx_buffer;
CANTXBufferType telem_can_tx_buffer;


void on_can1_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN1_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

void on_inverter_can_receive(const CAN_message_t &msg)
{
    TELEM_CAN.write(msg); // send immediately onto the telem CAN line
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    inverter_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
}

void on_telem_can_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    telem_can_rx_buffer.push_back(buf, sizeof(CAN_message_t));
}

namespace VCRCANInterfaceImpl
{

void vcr_CAN_recv(CANInterfaces& interfaces, const CAN_message_t& msg, unsigned long millis)
{
    switch(msg.id)
    {
        case PEDALS_SYSTEM_DATA_CANID:
        {
            interfaces.vcf_interface.receive_pedals_message(msg, millis);   
            break;
        }
        case DRIVEBRAIN_TORQUE_LIM_INPUT_CANID:
        {
            interfaces.db_interface.receive_drivebrain_torque_lim_command(msg, millis);
            break;
        } 
        case DRIVEBRAIN_SPEED_SET_INPUT_CANID:
        {
            interfaces.db_interface.receive_drivebrain_speed_command(msg, millis);
            break;
        }
        default:
        {
            break;
        }
        
    }
}

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface){
        CAN_message_t msg;
        while (buffer.available())
        {
            CAN_message_t msg;
            uint8_t buf[sizeof(CAN_message_t)];
            buffer.pop_front(buf, sizeof(CAN_message_t));
            memmove(&msg, buf, sizeof(msg));
            can_interface->write(msg);
        }
    }
}

