#ifdef ARDUINO
#include <Arduino.h>
#endif



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



/* Global definitions */
VCRInterfaceData_s interface_data;
VCRSystemData_s system_data;

MCP_ADC<8> adc_0 = MCP_ADC<8>(ADC0_CS);
MCP_ADC<8> adc_1 = MCP_ADC<8>(ADC1_CS);

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
    scheduler.schedule(kick_watchdog_task);
}

void loop() {
    scheduler.run();
}