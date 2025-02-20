#ifdef ARDUINO

#include <Arduino.h>
#endif

 // NOLINT for TaskScheduler



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"



/* Arduino specific upstream Libraries */
#include "QNEthernet.h"
#define _TASK_MICRO_RES // NOLINT
#include <TScheduler.hpp>

/* Local includes */
#include "VCR_Globals.h"
#include "VCR_Constants.h"
#include "VCR_Tasks.h"
#include "TorqueControllerMux.hpp"
#include "VCFInterface.h"

#include "VCRCANInterfaceImpl.h"
#include "FlexCAN_T4.h"


FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> INV_CAN;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> TELEM_CAN;

/* Scheduler setup */
TsScheduler task_scheduler;

constexpr unsigned long adc_sample_period_us = 250; 
// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use 
TsTask adc_0_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc0_task, &task_scheduler, false, &init_read_adc0_task);
TsTask adc_1_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc1_task, &task_scheduler, false, &init_read_adc1_task);

// /* Ethernet message sockets */ // TODO: Move this into its own interface
// qindesign::network::EthernetUDP protobuf_send_socket;
// qindesign::network::EthernetUDP protobuf_recv_socket;

// // InverterInterface mci1_interface = InverterInterface(&CAN1_rxBuffer, 1, 2);

void setup() {
    SPI.begin(); // TODO this should be elsewhere maybe
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, on_telem_can_receive);

    adc_0_sample_task.enable(); // will run the init function and allow the task to start running
    adc_1_sample_task.enable();
}

void loop() {
    task_scheduler.execute();
}
