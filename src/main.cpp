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
#include "VCFInterface.h"
#include "VCREthernetInterface.h"
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_InterfaceTasks.h"

#include "FlexCAN_T4.h"
#include "VCRCANInterfaceImpl.h"

#include "etl/singleton.h"

#include "DrivebrainInterface.h"
#include "InverterInterface.h"
#include "DrivetrainSystem.h"


// has to be included here as the define is only defined for source files in the implementation
// not in the library folder (makes sense)
#include "device_fw_version.h"  // from pio-git-hash


#include "EthernetAddressDefs.h"

FlexCAN_Type<CAN2> INV_CAN;
FlexCAN_Type<CAN3> TELEM_CAN;

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
/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

EthernetIPDefs_s car_network_definition;

InverterParams_s inverter_params = {
    .MINIMUM_HV_VOLTAGE = 400.0
};

InverterInterface fl_inverter_int(INV1_CONTROL_WORD_CANID, INV1_CONTROL_INPUT_CANID, INV1_CONTROL_PARAMETER_CANID, inverter_params);

DrivetrainSystem::InverterFuncts fl_inverter_functs = {
    .set_speed = [](float desired_rpm, float torque_limit_nm) { fl_inverter_int.set_speed(desired_rpm, torque_limit_nm);},
    .set_idle = []() { fl_inverter_int.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s control_word) { fl_inverter_int.set_inverter_control_word(control_word); },
    .get_status = []() { return fl_inverter_int.get_status(); },
    .get_motor_mechanics = []() { return fl_inverter_int.get_motor_mechanics(); }
};

veh_vec<DrivetrainSystem::InverterFuncts> inverter_functs(fl_inverter_functs, fl_inverter_functs, fl_inverter_functs, fl_inverter_functs);

DrivetrainSystem drivetrain_system(inverter_functs);

DrivetrainInit_s init = {DrivetrainModeRequest_e::INIT_DRIVE_MODE};

void setup() {
    vcr_data.fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    vcr_data.fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    vcr_data.fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

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

    adc_0_sample_task.enable(); // will run the init function and allow the task to start running
    adc_1_sample_task.enable();
    suspension_CAN_send.enable();
    CAN_send.enable();
    update_buzzer_controller_task.enable();
    kick_watchdog_task.enable();
    ethernet_send.enable();
    
}

void loop() { 
    drivetrain_system.evaluate_drivetrain(init);
    task_scheduler.execute(); 
}
