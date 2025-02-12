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
        default:
        {
            break;
        }
        
    }
}
}

