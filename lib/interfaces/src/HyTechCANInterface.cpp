#include <HyTechCANInterface.h>
#include <Arduino.h>

/* Recieve Buffers */
CANBufferType CAN1_rxBuffer;
CANBufferType CAN2_rxBuffer;
CANBufferType CAN3_rxBuffer;

/* Transfer Buffers */
CANBufferType CAN1_txBuffer;
CANBufferType CAN2_txBuffer;
CANBufferType CAN3_txBuffer;

/* Methods called on can recieve */
void on_can1_recieve(const CAN_message_t &msg) 
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN1_rxBuffer.push_back(buf, sizeof(CAN_message_t));
}

void on_can2_recieve(const CAN_message_t &msg) 
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN2_rxBuffer.push_back(buf, sizeof(CAN_message_t));

}

void on_can3_recieve(const CAN_message_t &msg) 
{
    uint8_t buf[sizeof(CAN_message_t)];
    memmove(buf, &msg, sizeof(msg));
    CAN3_rxBuffer.push_back(buf, sizeof(CAN_message_t));

}