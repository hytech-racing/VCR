#ifndef VCRCANINTERFACEIMPL_H
#define VCRCANINTERFACEIMPL_H

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
#include "ACUInterface.h"

#include "InverterInterface.h"

using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

/* RX buffers for CAN extern declarations*/

template <CAN_DEV_TABLE CAN_DEV> using FlexCAN_Type = FlexCAN_T4<CAN_DEV, RX_SIZE_256, TX_SIZE_32>;

// this is being done to send immediately from the inverter CAN line to the TELEM CAN every inverter

struct CANInterfaces {
    explicit CANInterfaces(
        VCFInterface &vcf_int, 
        ACUInterface &acu_int,
        DrivebrainInterface &db_int, 
        InverterInterface &fl_inv_int,
        InverterInterface &fr_inv_int,
        InverterInterface &rl_inv_int,
        InverterInterface &rr_inv_int
    )
        : vcf_interface(vcf_int), 
          acu_interface(acu_int),
          db_interface(db_int),
          fl_inverter_interface(fl_inv_int),
          fr_inverter_interface(fr_inv_int),
          rl_inverter_interface(rl_inv_int),
          rr_inverter_interface(rr_inv_int) {}

    VCFInterface &vcf_interface;
    ACUInterface &acu_interface;
    DrivebrainInterface &db_interface;
    InverterInterface &fl_inverter_interface;
    InverterInterface &fr_inverter_interface;
    InverterInterface &rl_inverter_interface;
    InverterInterface &rr_inverter_interface;

};

using CANInterfacesInstance = etl::singleton<CANInterfaces>;


namespace VCRCANInterfaceImpl {

    extern CANRXBufferType auxillary_can_rx_buffer;
    extern CANRXBufferType inverter_can_rx_buffer;
    extern CANRXBufferType telem_can_rx_buffer;

    extern CANTXBufferType auxillary_can_tx_buffer;
    extern CANTXBufferType inverter_can_tx_buffer;
    extern CANTXBufferType telem_can_tx_buffer;

    extern FlexCAN_Type<CAN2> AUXILLARY_CAN;
    extern FlexCAN_Type<CAN1> TELEM_CAN;
    extern FlexCAN_Type<CAN3> INVERTER_CAN;

    void on_auxillary_can_receive(const CAN_message_t &msg);
    void on_inverter_can_receive(const CAN_message_t &msg);
    void on_telem_can_receive(const CAN_message_t &msg);
    void vcr_CAN_recv(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);
    
}; // namespace VCRCANInterfaceImpl

#endif // VCRCANINTERFACEIMPL_H