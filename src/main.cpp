#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From C++ standard library */
#include <chrono>

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* From shared-systems-lib libdep */
#include "SysClock.h"

/* From shared-interfaces-lib libdep */


/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCR_Tasks.h"



/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

auto start_time = std::chrono::high_resolution_clock::now();
unsigned long stdMicros()
{
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count();
    return static_cast<unsigned long>(elapsed);
}



/* ADC setup */
// MCP_ADC<8> a1 = MCP_ADC<8>(ADC1_CS);



/* Ethernet message sockets */
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;



void setup() {
    scheduler.setTimingFunction(stdMicros);
}

void loop() {
    scheduler.run();
}