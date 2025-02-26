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

// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use

constexpr unsigned long adc_sample_period_us = 250;                  // 250 us = 4kHz
constexpr unsigned long update_buzzer_controller_period_us = 100000; // 100 000 us = 10 Hz
constexpr unsigned long kick_watchdog_period_us = 10000;             // 10 000 us = 100 Hz
constexpr unsigned long ams_update_period_us = 10000;                // 10 000 us = 100 Hz
constexpr unsigned long ethernet_update_period = 10000;
// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use
TsTask suspension_CAN_send(4000, TASK_FOREVER, &handle_enqueue_suspension_CAN_data, &task_scheduler,
                           false);
TsTask adc_0_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc0_task, &task_scheduler,
                         false, &init_read_adc0_task);
TsTask adc_1_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc1_task, &task_scheduler,
                         false, &init_read_adc1_task);
TsTask update_buzzer_controller_task(adc_sample_period_us, TASK_FOREVER,
                                     &run_update_buzzer_controller_task, &task_scheduler, false);
TsTask kick_watchdog_task(kick_watchdog_period_us, TASK_FOREVER, &run_kick_watchdog,
                          &task_scheduler, false, &create_watchdog);
TsTask ams_system_task(ams_update_period_us, TASK_FOREVER, &run_ams_system_task, &task_scheduler,
                       false, &init_ams_system_task);

TsTask CAN_send(TASK_IMMEDIATE, TASK_FOREVER, &handle_send_all_data, &task_scheduler, false);
TsTask ethernet_send(ethernet_update_period, TASK_FOREVER, &handle_send_VCR_ethernet_data,
                     &task_scheduler, false);
constexpr unsigned long IOExpander_sample_period_us = 250;
TsTask IOExpander_read_task(IOExpander_sample_period_us, TASK_FOREVER, &read_IOExpander, &task_scheduler, false, &create_IOExpander);

/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;


void setup() {
    SPI.begin(); // TODO this should be elsewhere maybe
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, on_telem_can_receive);

    adc_0_sample_task.enable(); // will run the init function and allow the task to start running
    adc_1_sample_task.enable();
    suspension_CAN_send.enable();
    CAN_send.enable();
    update_buzzer_controller_task.enable();
    kick_watchdog_task.enable();
    ethernet_send.enable();
    
    IOExpander_read_task.enable();
}

void loop() {
    task_scheduler.execute();
}