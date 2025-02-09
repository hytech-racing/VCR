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
extern CANRXBufferType CAN2_rxBuffer;
extern CANRXBufferType CAN3_rxBuffer;

/* TX buffer for CAN1 */
extern CANTXBufferType CAN1_txBuffer;
/* TX buffer for CAN2 */
extern CANTXBufferType CAN2_txBuffer;
/* TX buffer for CAN3 */
extern CANTXBufferType CAN3_txBuffer;

void on_can1_receive(const CAN_message_t &msg);
void on_can2_receive(const CAN_message_t &msg);
void on_can3_receive(const CAN_message_t &msg);


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