#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From C++ standard library */
#include <chrono>

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* From shared-systems-lib libdep */
#include "SysClock.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCR_Globals.h"
#include "VCR_Constants.h"
#include "VCR_Tasks.h"
#include "TorqueControllerMux.hpp"


/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();



/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;



void setup() {
    scheduler.setTimingFunction(micros);

    scheduler.schedule(tick_state_machine_task);
    scheduler.schedule(read_adc0_task);
    scheduler.schedule(read_adc1_task);
    scheduler.schedule(update_buzzer_controller_task);
}

void loop() {
    scheduler.run();
}