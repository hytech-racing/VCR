#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCR_Globals.h"
#include "VCR_Constants.h"
#include "VCR_Tasks.h"
#include "TorqueControllerMux.hpp"
#include "VCREthernetInterface.h"
#include "VCFInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "FlexCAN_T4.h"


FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> INV_CAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> TELEM_CAN;
/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();



/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;


void setup() {
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, on_telem_can_receive);

    scheduler.setTimingFunction(micros);

    scheduler.schedule(read_adc0_task);
    scheduler.schedule(read_adc1_task);
    scheduler.schedule(update_buzzer_controller_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(update_ams_system_task);
}

void loop() {
    scheduler.run();
}