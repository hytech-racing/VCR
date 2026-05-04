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

/* Type aliases to improve readability, and provide one place to change parameters */
using CANRXBufferType = Circular_Buffer<uint8_t, (uint32_t)16, sizeof(CAN_message_t)>;
using CANTXBufferType = Circular_Buffer<uint8_t, (uint32_t)128, sizeof(CAN_message_t)>;

using TelemCAN_t = FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>;
using RearAuxCAN_t = FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16>;
using InverterCAN_t = FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>;

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

struct VCRCANInterface {
    VCRCANInterface(etl::delegate<void (CANInterfaces &, const CAN_message_t &, unsigned long, CANInterfaceType_e)> recv_switch_func)
        : can_recv_switch(recv_switch_func)
        {}
    
    TelemCAN_t TELEM_CAN;

    CANTXBufferType telem_can_tx_buffer;
    CANRXBufferType telem_can_rx_buffer;

    RearAuxCAN_t REAR_AUX_CAN;

    CANTXBufferType rear_aux_can_tx_buffer;
    CANRXBufferType rear_aux_can_rx_buffer;

    InverterCAN_t INVERTER_CAN;

    CANTXBufferType inverter_can_tx_buffer;
    CANRXBufferType inverter_can_rx_buffer;

    etl::delegate<void (CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type)> can_recv_switch;
};
using VCRCANInterfaceInstace = etl::singleton<VCRCANInterface>;

namespace VCRCANInterfaceImpl {
    void on_auxillary_can_receive(const CAN_message_t &msg);
    void on_inverter_can_receive(const CAN_message_t &msg);
    void on_telem_can_receive(const CAN_message_t &msg);
    void vcr_CAN_recv_switch(CANInterfaces &interfaces, const CAN_message_t &msg, unsigned long millis, CANInterfaceType_e interface_type);

    void send_all_CAN_msgs(CANTXBufferType &buffer, FlexCAN_T4_Base *can_interface);
}; // namespace VCRCANInterfaceImpl

#endif // VCRCANINTERFACEIMPL_H