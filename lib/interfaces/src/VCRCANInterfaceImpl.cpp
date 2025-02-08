#include "VCRCANInterfaceImpl.h"

#include "hytech.h"
#include <cstdint>
// // global forwards
CANRXBufferType CAN1_rxBuffer;
CANRXBufferType CAN2_rxBuffer;
CANRXBufferType CAN3_rxBuffer;

CANTXBufferType CAN1_txBuffer;
CANTXBufferType CAN2_txBuffer;
CANTXBufferType CAN3_txBuffer;

void on_can1_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN1_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

void on_can2_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN2_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

void on_can3_receive(const CAN_message_t &msg)
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN3_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

namespace VCRCANInterfaceImpl
{

void vcr_CAN_recv(CANInterfaces& interfaces, const CAN_message_t& msg, unsigned long millis)
{
    switch(msg.id)
    {
        case VCR_PEDALS_CANID:
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

