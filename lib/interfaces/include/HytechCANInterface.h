#ifndef HYTECHCANINTERFACE
#define HYTECHCANINTERFACE

#include <FlexCAN_T4.h>
#include <hytech.h>

template <uint32_t... can_ids>
class HytechCANInterface {
    public: 

        HytechCANInterface() { }

        /** 
         * Registers CAN id to the handlers delegate
         */
        template <uint16_t can_id>
        void register_handler(void (callback)(CAN_message_t &msg))
        {
            if constexpr ((can_id == can_ids) || ...) {
                handlers[CAN_ID] = etl::delegate<void(const uint8_t*, size_t)>::create(callback);
            } else {
                static_assert(((can_id ==can_ids) || ...), "CAN id not registered.")
            }
        }

        /**
         * Dispatches CAN messages from the specified buffer 
         * (Should be called in a loop)
         */
        template <typename bufferType>
        void dispatch_buffer(bufferType &rx_buffer) 
        {
            while (rx_buffer.available())
            {
                CAN_message_t recvd_msg;
                uint8_t buf[sizeof(CAN_message_t)];
                rx_buffer.pop_front(buf, sizeof(CAN_message_t));
                memmove(&recvd_msg, buf, sizeof(recvd_msg));
                dispatch_msg(recvd_msg.id);
            }
        }

        /**
         * Sending CAN messages 
         * (needs to be called in loop)
         * */
        template <typename bufferType> 
        void send_all_CAN_msgs(bufferType &buffer, FlexCAN_T4_Base *can_interface)
        {
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

        /**
         * Recieve Buffers
         */

        /* Recieve buffer for CAN 1 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN1_rxBuffer;

        /* Recieve buffer for CAN 2 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN2_rxBuffer;

        /* Recieve buffer for CAN 3 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN3_rxBuffer;

        /**
         * Transfer buffers
         */

        /* Transfer buffer for CAN 1 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN1_txBuffer;

        /* Transfer buffer for CAN 2 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN2_txBuffer;

        /* Transfer buffer for CAN 3 */
        Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)> CAN3_txBuffer;

    private: 

        /**
         * Tries to dispatch CAN message out to the correct handler
         * if there are more than 0 CAN ids
         */
        void dispatch_msg(CAN_message_t &msg)
        {   
            if constexpr (sizeof...(can_ids) > 0) {
                (void)((can_id == can_ids && handlers[msg.id](msg)) || ...);
            }
        }

        /**
         * Stores recieve callbacks
         */
        etl::delegate<void(CAN_message_t &msg)> handlers[sizeof...(can_ids)] = {};

};

#endif /* HYTECHCANINTERFACE */