#ifdef ARDUINO
#include <Arduino.h>
#endif

/* From C++ standard library */
#include <chrono>
#include <stdint.h>

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

#include <InverterInterface.h>
#include <HytechCANInterface.h>

#include <hytech.h>

#include <hytech.h>

// /* Scheduler setup */
// HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

// /* Ethernet message sockets */ // TODO: Move this into its own interface
// qindesign::network::EthernetUDP protobuf_send_socket;
// qindesign::network::EthernetUDP protobuf_recv_socket;

// // InverterInterface mci1_interface = InverterInterface(&CAN1_rxBuffer, 1, 2);

// void setup() {
//     scheduler.setTimingFunction(micros);

//     scheduler.schedule(tick_state_machine_task);
//     scheduler.schedule(read_adc0_task);
//     scheduler.schedule(read_adc1_task);
//     scheduler.schedule(update_buzzer_controller_task);
// }

// void loop() {
//     scheduler.run();
// }

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> TEST_CAN1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> TEST_CAN2; // Inverter CAN (now both are on same line)
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> TEST_CAN3; // Inverter CAN (now both are on same line)

CAN_message_t msg;

using CircularBufferType = CANBufferType;
using InverterInterfaceType = InverterInterface;

InverterInterfaceType fl_inv = InverterInterfaceType(&CAN3_txBuffer, MC1_SETPOINTS_COMMAND_CANID, MC1_TORQUE_COMMAND_CANID);
InverterInterfaceType fr_inv = InverterInterfaceType(&CAN3_txBuffer, MC2_SETPOINTS_COMMAND_CANID, MC2_TORQUE_COMMAND_CANID);
InverterInterfaceType rl_inv = InverterInterfaceType(&CAN3_txBuffer, MC3_SETPOINTS_COMMAND_CANID, MC3_TORQUE_COMMAND_CANID);
InverterInterfaceType rr_inv = InverterInterfaceType(&CAN3_txBuffer, MC4_SETPOINTS_COMMAND_CANID, MC4_TORQUE_COMMAND_CANID);

CANInterfaces CAN_interfaces = {fl_inv, fr_inv, rl_inv, rr_inv};

void init_can_interface()
{
    TEST_CAN2.begin();
    TEST_CAN2.setBaudRate(500000);
    TEST_CAN2.setMaxMB(16);
    TEST_CAN2.enableFIFO();
    TEST_CAN2.enableFIFOInterrupt();
    TEST_CAN2.onReceive(on_can2_recieve);
    TEST_CAN2.mailboxStatus();

    TEST_CAN1.begin();
    TEST_CAN1.setBaudRate(500000);
    TEST_CAN1.setMaxMB(16);
    TEST_CAN1.enableFIFO();
    TEST_CAN1.enableFIFOInterrupt();
    TEST_CAN1.enableMBInterrupts();
    TEST_CAN1.onReceive(on_can1_recieve);
    TEST_CAN1.mailboxStatus();

    TEST_CAN3.begin();
    TEST_CAN3.setBaudRate(500000);
    TEST_CAN3.setMaxMB(16);
    TEST_CAN3.enableFIFO();
    TEST_CAN3.enableFIFOInterrupt();
    TEST_CAN3.enableMBInterrupts();
    TEST_CAN3.onReceive(on_can3_recieve);
    TEST_CAN3.mailboxStatus();
}

void setup(void)
{
    while (!Serial)
    {
        // wait for Arduino Serial Monitor to be ready
    }
    init_can_interface();
}
bool ran_test = false;
void loop()
{

    // TEST SEND

    Serial.println("=== TESTING SEND ===");

    Serial.println("Adding torque command to tx buffer...");

    fl_inv.send_MC_TORQUE_COMMAND();

    Serial.print("Tx buffer size (should be 1) ");
    Serial.println(CAN3_txBuffer.size());

    Serial.println("Sending CAN messages via can interface...");
    send_all_CAN_msgs(CAN3_txBuffer, &TEST_CAN3);

    Serial.print("Tx buffer size (should be 0): ");
    Serial.println(CAN3_txBuffer.size());

    delay(100);

    Serial.println("");

    // Serial.println("=== TESTING RECIEVE ===");

    // Serial.print("Rx buffer size before processing: ");
    // Serial.println(CAN3_rxBuffer.size());

    // process_ring_buffer<CircularBufferType>(CAN3_rxBuffer, CAN_interfaces);

    // Serial.print("Rx buffer size after processing (shoud be 0): ");
    // Serial.println(CAN3_rxBuffer.size());

    // Serial.println("");
    // Serial.println("");



    // for (uint8_t i = 0; i < 8; i++)
    //     msg.buf[i] = i + 1;
    // TEST_CAN3.write(msg);
    // ran_test = true;
    // delay(10);
    // Serial.println("======= TEST 1 =======");
    // Serial.println("can3 rx buffer size: (should be 1)");
    // Serial.println(CAN3_rxBuffer.available());
    // Serial.println("======= TEST 2 =======");
    // Serial.println("can1 received message equal to sent msg");
    // delay(10);
    // if (CAN1_rxBuffer.available())
    // {
    //     CAN_message_t recvd_msg;
    //     uint8_t buf[sizeof(CAN_message_t)];
    //     CAN1_rxBuffer.pop_front(buf, sizeof(CAN_message_t));
    //     memmove(&recvd_msg, buf, sizeof(recvd_msg));

    //     Serial.println("\tid = prev: (should be 1)");
    //     Serial.print("\t");
    //     Serial.println(recvd_msg.id == msg.id);
    // }
}
