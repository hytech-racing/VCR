#ifdef ARDUINO
#include <Arduino.h>
#endif

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* From shared-systems-lib libdep */
#include "SysClock.h"

/* From Arduino Libraries */
#include "QNEthernet.h"
using namespace qindesign::network;


/* */
EthernetUDP protobuf_send_socket;
EthernetUDP protobuf_recv_socket;

void setup() {
    SysTick_s tick = {0, 0, {0, 0, 0, 0, 0, 0, 1}}; // 1Hz trigger
}

void loop() {
    
}