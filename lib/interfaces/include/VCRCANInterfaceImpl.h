#ifndef __VCRCANINTERFACEIMPL_H__
#define __VCRCANINTERFACEIMPL_H__

#include <cstdint>

#include <tuple>
#include <utility>

#include "FlexCAN_T4.h"

#include "etl/delegate.h"
#include "etl/singleton.h"

#include "CANInterface.h"

#include "SharedFirmwareTypes.h"

#include "hytech.h" // generated CAN library
#include "shared_types.h"

#include "DrivebrainInterface.h"
#include "VCFInterface.h"

#include "InverterInterface.h"

using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/

template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_16>;

// this is being done to send immediately from the inverter CAN line to the TELEM CAN every inverter

struct CANInterfaces {
    explicit CANInterfaces(
        VCFInterface &vcf_int, 
        DrivebrainInterface &db_int, 
        InverterInterface &fl_inv_int,
        InverterInterface &fr_inv_int,
        InverterInterface &rl_inv_int,
        InverterInterface &rr_inv_int
    )
        : vcf_interface(vcf_int), 
          db_interface(db_int),
          fl_inverter_interface(fl_inv_int),
          fr_inverter_interface(fr_inv_int),
          rl_inverter_interface(rl_inv_int),
          rr_inverter_interface(rr_inv_int) {}

    VCFInterface &vcf_interface;
    DrivebrainInterface &db_interface;
    InverterInterface &fl_inverter_interface;
    InverterInterface &fr_inverter_interface;
    InverterInterface &rl_inverter_interface;
    InverterInterface &rr_inverter_interface;

};

using CANInterfacesInstance = etl::singleton<CANInterfaces>;

namespace VCRCANInterfaceImpl {

extern CANRXBufferType CAN1_rxBuffer;
extern CANRXBufferType inverter_can_rx_buffer;
extern CANRXBufferType telem_can_rx_buffer;

/* TX buffer for CAN1 */
extern CANTXBufferType CAN1_txBuffer;
/* TX buffer for CAN2 */
extern CANTXBufferType inverter_can_tx_buffer;
/* TX buffer for CAN3 */
extern CANTXBufferType telem_can_tx_buffer;
extern FlexCAN_Type<CAN3> TELEM_CAN; // gets defined in main as of right now

void on_can1_receive(const CAN_message_t &msg);
void on_inverter_can_receive(const CAN_message_t &msg);
void on_telem_can_receive(const CAN_message_t &msg);
void vcr_CAN_recv(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis);

void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);
}; // namespace VCRCANInterfaceImpl

#endif // __VCRCANINTERFACEIMPL_H__