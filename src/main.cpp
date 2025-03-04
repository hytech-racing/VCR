#ifdef ARDUINO

#include <Arduino.h>
#endif

 // NOLINT for TaskScheduler

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* Arduino specific upstream Libraries */
#include "QNEthernet.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

#define _TASK_MICRO_RES // NOLINT
#include <TScheduler.hpp>

/* Local includes */
#include "TorqueControllerMux.hpp"
#include "VCFInterface.h"
#include "VCREthernetInterface.h"
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_InterfaceTasks.h"

#include "FlexCAN_T4.h"
#include "VCRCANInterfaceImpl.h"

#include "etl/singleton.h"

#include "DrivebrainInterface.h"


// has to be included here as the define is only defined for source files in the implementation
// not in the library folder (makes sense)
#include "device_fw_version.h"  // from pio-git-hash


#include "EthernetAddressDefs.h"

FlexCAN_Type<CAN2> INV_CAN;
FlexCAN_Type<CAN3> TELEM_CAN;

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();


// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use

constexpr unsigned long adc0_sample_period_us = 250;                 // 250 us = 4 kHz
constexpr unsigned long adc1_sample_period_us = 10000;               // 10000 us = 100 Hz
constexpr unsigned long update_buzzer_controller_period_us = 100000; // 100 000 us = 10 Hz
constexpr unsigned long kick_watchdog_period_us = 10000;             // 10 000 us = 100 Hz
constexpr unsigned long ams_update_period_us = 10000;                // 10 000 us = 100 Hz
constexpr unsigned long ethernet_update_period = 10000;              // 10 000 us = 100 Hz
constexpr unsigned long suspension_can_period_us = 4000;             // 4000 us = 250 Hz
constexpr unsigned long IOexpander_sample_period_us = 50000;         // 50000 us = 20 Hz

// task declarations
HT_TASK::Task adc_0_sample_task(init_read_adc0_task, run_read_adc0_task, 5, adc0_sample_period_us);
HT_TASK::Task adc_1_sample_task(init_read_adc1_task, run_read_adc1_task, 50, adc1_sample_period_us);
HT_TASK::Task update_buzzer_controller_task(HT_TASK::DUMMY_FUNCTION, run_update_buzzer_controller_task, 3, update_buzzer_controller_period_us);
HT_TASK::Task kick_watchdog_task(init_kick_watchdog, run_kick_watchdog, 0, kick_watchdog_period_us); 
HT_TASK::Task ams_system_task(init_ams_system_task, run_ams_system_task, 2, ams_update_period_us);
HT_TASK::Task suspension_CAN_send(HT_TASK::DUMMY_FUNCTION, handle_enqueue_suspension_CAN_data, 4, suspension_can_period_us);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, handle_send_all_data, 5);
HT_TASK::Task ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, 6, ethernet_update_period);
HT_TASK::Task IOExpander_read_task(init_ioexpander, read_ioexpander, 100, IOexpander_sample_period_us);
/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

EthernetIPDefs_s car_network_definition;

void setup() {
    vcr_data.fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    vcr_data.fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    vcr_data.fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

    // timing function
    scheduler.setTimingFunction(micros);

    qindesign::network::Ethernet.begin(
        car_network_definition.vcr_ip, car_network_definition.default_dns,
        car_network_definition.default_gateway, car_network_definition.car_subnet);
    protobuf_send_socket.begin(car_network_definition.VCRData_port);
    DrivebrainInterfaceInstance::create(vcr_data.interface_data.rear_loadcell_data,
                                        vcr_data.interface_data.rear_suspot_data,
                                        car_network_definition.drivebrain_ip,
                                        car_network_definition.VCRData_port, &protobuf_send_socket);
    SPI.begin(); // TODO this should be elsewhere maybe
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INV_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_telem_can_receive);

// will run the init functions and allow the tasks to start running
    scheduler.schedule(adc_0_sample_task);
    scheduler.schedule(adc_1_sample_task);
    scheduler.schedule(update_buzzer_controller_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(suspension_CAN_send);
    scheduler.schedule(CAN_send);
    scheduler.schedule(ethernet_send);
    scheduler.schedule(IOExpander_read_task);
}

void loop() { scheduler.run(); }
