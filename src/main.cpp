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
#include "TorqueControllerMux.hpp"
#include "VCREthernetInterface.h"
#include "VCFInterface.h"
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_InterfaceTasks.h"

#include "VCRCANInterfaceImpl.h"
#include "FlexCAN_T4.h"
#include "VCRCANInterfaceImpl.h"

#include "etl/singleton.h"

#include "DrivebrainInterface.h"

// class DrivebrainInterface;

FlexCAN_Type<CAN2> INV_CAN;
FlexCAN_Type<CAN3> TELEM_CAN;

/* Scheduler setup */
TsScheduler task_scheduler;


// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use 

constexpr unsigned long adc_sample_period_us = 250; // 250 us = 4kHz
constexpr unsigned long update_buzzer_controller_period_us = 100000;  // 100 000 us = 10 Hz
constexpr unsigned long kick_watchdog_period_us = 10000;  // 10 000 us = 100 Hz
constexpr unsigned long ams_update_period_us = 10000;  // 10 000 us = 100 Hz
// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use 
TsTask suspension_CAN_send(4000, TASK_FOREVER, &handle_enqueue_suspension_CAN_data, &task_scheduler, false);
TsTask adc_0_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc0_task, &task_scheduler, false, &init_read_adc0_task);
TsTask adc_1_sample_task(adc_sample_period_us, TASK_FOREVER, &run_read_adc1_task, &task_scheduler, false, &init_read_adc1_task);
TsTask update_buzzer_controller_task(adc_sample_period_us, TASK_FOREVER, &run_update_buzzer_controller_task, &task_scheduler, false);
TsTask kick_watchdog_task(kick_watchdog_period_us, TASK_FOREVER, &run_kick_watchdog, &task_scheduler, false, &create_watchdog);
TsTask ams_system_task(ams_update_period_us, TASK_FOREVER, &run_ams_system_task, &task_scheduler, false, &init_ams_system_task);

TsTask CAN_send(TASK_IMMEDIATE, TASK_FOREVER, &handle_send_all_data, &task_scheduler, false);
/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

void setup() {
    // TODO 
    DrivebrainInterfaceInstance::create(vcr_data.interface_data.rear_loadcell_data, vcr_data.interface_data.rear_suspot_data); 
    SPI.begin(); // TODO this should be elsewhere maybe
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_telem_can_receive);

    adc_0_sample_task.enable(); // will run the init function and allow the task to start running
    adc_1_sample_task.enable();
    suspension_CAN_send.enable();
    CAN_send.enable();
    update_buzzer_controller_task.enable();
    kick_watchdog_task.enable();
}

void loop() { task_scheduler.execute(); }
