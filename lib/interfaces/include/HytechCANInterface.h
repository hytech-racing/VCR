#ifndef HYTECHCANINTERFACE
#define HYTECHCANINTERFACE

#include <FlexCAN_T4.h>
#include <hytech.h>
#include <etl/delegate.h>
#include <stdint.h>
#include <InverterInterface.h>
#include <MessageQueueDefine.h>

/* Recieve Buffers */
extern CANBufferType CAN1_rxBuffer;
extern CANBufferType CAN2_rxBuffer;
extern CANBufferType CAN3_rxBuffer;

/* Transfer Buffers */
extern CANBufferType CAN1_txBuffer;
extern CANBufferType CAN2_txBuffer;
extern CANBufferType CAN3_txBuffer;

/**
 * Struct holding the interfaces processed by the ring buffer
 */

struct CANInterfaces
{
    InverterInterface front_left_inv;
    InverterInterface front_right_inv;
    InverterInterface back_left_inv;
    InverterInterface back_right_inv;
};

/** Methods called on can recieve */
void on_can1_recieve(const CAN_message_t &msg); 

void on_can2_recieve(const CAN_message_t &msg); 

void on_can3_recieve(const CAN_message_t &msg); 

/**
 * Recieving CAN messages
 * (needs to be called in a loop)
 */
template <typename BufferType>
void process_ring_buffer(BufferType &rx_buffer, CANInterfaces interfaces) {
    while (rx_buffer.available()) 
    {
        CAN_message_t recvd_msg;
        uint8_t buf[sizeof(CAN_message_t)];
        rx_buffer.pop_front(buf, sizeof(CAN_message_t));
        memmove(&recvd_msg, buf, sizeof(recvd_msg));
        switch (recvd_msg.id)
        {
            // FL Inverter
            case (MCI1_STATUS_CANID):
                interfaces.front_left_inv.recieve_MCI_STATUS(recvd_msg);
                break;

            case (MCI1_TEMPS_CANID):
                interfaces.front_left_inv.recieve_MCI_TEMPS(recvd_msg);
                break;

            case (MCI1_DYNAMICS_CANID):
                interfaces.front_left_inv.recieve_MCI_DYNAMICS(recvd_msg);
                break;

            case (MCI1_POWER_CANID):
                interfaces.front_left_inv.recieve_MCI_POWER(recvd_msg);
                break;

            case (MCI1_FEEDBACK_CANID):
                interfaces.front_left_inv.recieve_MCI_FEEDBACK(recvd_msg);

            // FR inverter
            case (MCI2_STATUS_CANID):
                interfaces.front_right_inv.recieve_MCI_STATUS(recvd_msg);
                break;

            case (MCI2_TEMPS_CANID):
                interfaces.front_right_inv.recieve_MCI_TEMPS(recvd_msg);
                break;

            case (MCI2_DYNAMICS_CANID):
                interfaces.front_right_inv.recieve_MCI_DYNAMICS(recvd_msg);
                break;

            case (MCI2_POWER_CANID):
                interfaces.front_right_inv.recieve_MCI_POWER(recvd_msg);
                break;
                
            case (MCI2_FEEDBACK_CANID):
                interfaces.front_right_inv.recieve_MCI_FEEDBACK(recvd_msg);

            // RL Inverter
            case (MCI3_STATUS_CANID):
                interfaces.back_left_inv.recieve_MCI_STATUS(recvd_msg);
                break;

            case (MCI3_TEMPS_CANID):
                interfaces.back_left_inv.recieve_MCI_TEMPS(recvd_msg);
                break;

            case (MCI3_DYNAMICS_CANID):
                interfaces.back_left_inv.recieve_MCI_DYNAMICS(recvd_msg);
                break;

            case (MCI3_POWER_CANID):
                interfaces.back_left_inv.recieve_MCI_POWER(recvd_msg);
                break;

            case (MCI3_FEEDBACK_CANID):
                interfaces.back_left_inv.recieve_MCI_FEEDBACK(recvd_msg);

            // RR Inverter
            case (MCI4_STATUS_CANID):
                interfaces.back_right_inv.recieve_MCI_STATUS(recvd_msg);
                break;

            case (MCI4_TEMPS_CANID):
                interfaces.back_right_inv.recieve_MCI_TEMPS(recvd_msg);
                break;

            case (MCI4_DYNAMICS_CANID):
                interfaces.back_right_inv.recieve_MCI_DYNAMICS(recvd_msg);
                break;

            case (MCI4_POWER_CANID):
                interfaces.back_right_inv.recieve_MCI_POWER(recvd_msg);
                break;

            case (MCI4_FEEDBACK_CANID):
                interfaces.back_right_inv.recieve_MCI_FEEDBACK(recvd_msg);

        }
    
    }
}

/**
* Sending CAN messages 
* (needs to be called in loop)
* */
template <typename bufferType> 
void send_all_CAN_msgs(bufferType &tx_buffer, FlexCAN_T4_Base *can_interface)
{
    CAN_message_t msg;
    while (tx_buffer.available()) 
    {
        CAN_message_t msg;
        uint8_t buf[sizeof(CAN_message_t)];
        tx_buffer.pop_front(buf, sizeof(CAN_message_t));
        memmove(&msg, buf, sizeof(msg));
        can_interface->write(msg);
    }
}

#endif /* HYTECHCANINTERFACE */