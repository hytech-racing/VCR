#ifndef __VCRCANINTERFACEIMPL_H__
#define __VCRCANINTERFACEIMPL_H__

#include <cstdint>

#include <tuple>
#include <utility>

#include "FlexCAN_T4.h"

#include "etl/delegate.h"

#include "CANInterface.h"


#include "InverterInterface.h"

#include "SharedFirmwareTypes.h"

#include "shared_types.h"
#include "hytech.h" // generated CAN library

#include "VCFInterface.h"

using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/
extern CANRXBufferType CAN1_rxBuffer;
extern CANRXBufferType inverter_can_rx_buffer;
extern CANRXBufferType telem_can_rx_buffer;

/* TX buffer for CAN1 */
extern CANTXBufferType CAN1_txBuffer;
/* TX buffer for CAN2 */
extern CANTXBufferType inverter_can_tx_buffer;
/* TX buffer for CAN3 */
extern CANTXBufferType telem_can_tx_buffer;

void on_can1_receive(const CAN_message_t &msg);
void on_inverter_can_receive(const CAN_message_t &msg);
void on_telem_can_receive(const CAN_message_t &msg);


struct CANInterfaces
{
    explicit CANInterfaces(VCFInterface& vcf_int): vcf_interface(vcf_int) {}

    VCFInterface& vcf_interface;
};

namespace VCRCANInterfaceImpl
{
    void vcr_CAN_recv(CANInterfaces& interfaces, const CAN_message_t& msg, unsigned long millis);
};


#endif // __VCRCANINTERFACEIMPL_H__