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
#include "InverterInterface.h"
#include "DrivetrainSystem.h"
#include "VCR_SystemTasks.h"



// has to be included here as the define is only defined for source files in the implementation
// not in the library folder (makes sense)
#include "device_fw_version.h"  // from pio-git-hash


#include "EthernetAddressDefs.h"

#include "VehicleStateMachine.h"



FlexCAN_Type<CAN3> TELEM_CAN;
FlexCAN_Type<CAN2> INVERTER_CAN;
/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

bool handle_big_tasks(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);


// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use

constexpr unsigned long adc0_sample_period_us = 250;                 // 250 us = 4 kHz
constexpr unsigned long adc1_sample_period_us = 10000;               // 10000 us = 100 Hz
constexpr unsigned long update_buzzer_controller_period_us = 100000; // 100 000 us = 10 Hz
constexpr unsigned long kick_watchdog_period_us = 10000;             // 10 000 us = 100 Hz
constexpr unsigned long ams_update_period_us = 10000;                // 10 000 us = 100 Hz
constexpr unsigned long ethernet_update_period = 10000;              // 10 000 us = 100 Hz
constexpr unsigned long suspension_can_period_us = 4000;             // 4000 us = 250 Hz
constexpr unsigned long inv_send_period = 4000;             // 4 000 us = 250 Hz
constexpr unsigned long ioexpander_sample_period_us = 50000;

// task declarations
HT_TASK::Task adc_0_sample_task([](const unsigned long&, const HT_TASK::TaskInfo&) { return true; }, run_read_adc0_task, 5, adc0_sample_period_us);
HT_TASK::Task adc_1_sample_task([](const unsigned long&, const HT_TASK::TaskInfo&) { return true; }, run_read_adc1_task, 50, adc1_sample_period_us);
HT_TASK::Task update_buzzer_controller_task(HT_TASK::DUMMY_FUNCTION, run_update_buzzer_controller_task, 3, update_buzzer_controller_period_us);
HT_TASK::Task kick_watchdog_task(init_kick_watchdog, run_kick_watchdog, 0, kick_watchdog_period_us); 
HT_TASK::Task ams_system_task(init_ams_system_task, run_ams_system_task, 2, ams_update_period_us);
HT_TASK::Task suspension_CAN_send(HT_TASK::DUMMY_FUNCTION, handle_enqueue_suspension_CAN_data, 4, suspension_can_period_us);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, handle_send_all_data, 5);

HT_TASK::Task ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, 6);
HT_TASK::Task IOExpander_read_task(init_ioexpander, read_ioexpander, 100, ioexpander_sample_period_us);
HT_TASK::Task inverter_CAN_send(HT_TASK::DUMMY_FUNCTION, handle_inverter_CAN_send, 5, inv_send_period);
HT_TASK::Task big_task_t(HT_TASK::DUMMY_FUNCTION, handle_big_tasks, 0);


/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

EthernetIPDefs_s car_network_definition;

InverterParams_s inverter_params = {
    .MINIMUM_HV_VOLTAGE = 400.0
};

// Inverter Interfaces
InverterInterface fl_inverter_int(INV1_CONTROL_WORD_CANID, INV1_CONTROL_INPUT_CANID, INV1_CONTROL_PARAMETER_CANID, inverter_params);
InverterInterface fr_inverter_int(INV2_CONTROL_WORD_CANID, INV2_CONTROL_INPUT_CANID, INV2_CONTROL_PARAMETER_CANID, inverter_params);
InverterInterface rl_inverter_int(INV3_CONTROL_WORD_CANID, INV3_CONTROL_INPUT_CANID, INV3_CONTROL_PARAMETER_CANID, inverter_params);
InverterInterface rr_inverter_int(INV4_CONTROL_WORD_CANID, INV4_CONTROL_INPUT_CANID, INV4_CONTROL_PARAMETER_CANID, inverter_params);

// Inverter Functs
DrivetrainSystem::InverterFuncts fl_inverter_functs = {
    .set_speed = [](float desired_rpm, float torque_limit_nm) { fl_inverter_int.set_speed(desired_rpm, torque_limit_nm);},
    .set_idle = []() { fl_inverter_int.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s control_word) { fl_inverter_int.set_inverter_control_word(control_word); },
    .get_status = []() { return fl_inverter_int.get_status(); },
    .get_motor_mechanics = []() { return fl_inverter_int.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts fr_inverter_functs = {
    .set_speed = [](float desired_rpm, float torque_limit_nm) { fr_inverter_int.set_speed(desired_rpm, torque_limit_nm);},
    .set_idle = []() { fr_inverter_int.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s control_word) { fr_inverter_int.set_inverter_control_word(control_word); },
    .get_status = []() { return fr_inverter_int.get_status(); },
    .get_motor_mechanics = []() { return fr_inverter_int.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts rl_inverter_functs = {
    .set_speed = [](float desired_rpm, float torque_limit_nm) { rl_inverter_int.set_speed(desired_rpm, torque_limit_nm);},
    .set_idle = []() { rl_inverter_int.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s control_word) { rl_inverter_int.set_inverter_control_word(control_word); },
    .get_status = []() { return rl_inverter_int.get_status(); },
    .get_motor_mechanics = []() { return rl_inverter_int.get_motor_mechanics(); }
};

DrivetrainSystem::InverterFuncts rr_inverter_functs = {
    .set_speed = [](float desired_rpm, float torque_limit_nm) { rr_inverter_int.set_speed(desired_rpm, torque_limit_nm);},
    .set_idle = []() { rr_inverter_int.set_idle(); },
    .set_inverter_control_word = [](InverterControlWord_s control_word) { rr_inverter_int.set_inverter_control_word(control_word); },
    .get_status = []() { return rr_inverter_int.get_status(); },
    .get_motor_mechanics = []() { return rr_inverter_int.get_motor_mechanics(); }
};

veh_vec<DrivetrainSystem::InverterFuncts> inverter_functs(fl_inverter_functs, fr_inverter_functs, rl_inverter_functs, rr_inverter_functs);

// Drivetrain system stuff
DrivetrainSystem drivetrain_system(inverter_functs);

VCFInterface vcf_interface;

VCRAsynchronousInterfaces vcr_async_interfaces(CANInterfacesInstance::instance());

etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

VCRInterfaceData_s int_data; // TODO what were we planning on doing with this

VehicleStateMachine vehicle_statemachine = VehicleStateMachine(
    etl::delegate<bool()>::create([]() { return true; }), 
    etl::delegate<bool()>::create([]() { return true; }), 
    etl::delegate<bool()>::create([]() { return true; }), 
    etl::delegate<bool()>::create([]() { return true; }), 
    etl::delegate<bool()>::create([]() { return true; }),
    etl::delegate<void()>::create([]() { }),
    etl::delegate<bool()>::create([]() { return true; }), 
    etl::delegate<void()>::create([]() { }), 
    etl::delegate<void()>::create([]() { }), 
    etl::delegate<void()>::create([]() { })
);

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

    CANInterfacesInstance::create(
        vcf_interface,
        DrivebrainInterfaceInstance::instance(), 
        fl_inverter_int,
        fr_inverter_int,
        rl_inverter_int,
        rr_inverter_int
    );
        
    SPI.begin(); // TODO this should be elsewhere maybe
    init_bundle();
    const uint32_t CAN_baudrate = 500000;
    // from CANInterfaceon_inverter_can_receive
    handle_CAN_setup(INVERTER_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(TELEM_CAN, CAN_baudrate, VCRCANInterfaceImpl::on_telem_can_receive);

    scheduler.schedule(adc_0_sample_task);
    scheduler.schedule(adc_1_sample_task);
    scheduler.schedule(update_buzzer_controller_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(suspension_CAN_send);
    scheduler.schedule(CAN_send);
    scheduler.schedule(ethernet_send);
    scheduler.schedule(inverter_CAN_send);
    scheduler.schedule(big_task_t);
    scheduler.schedule(IOExpander_read_task);
}

void loop() { 
    scheduler.run(); 
}

bool handle_big_tasks(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    big_task(main_can_recv, vcr_async_interfaces, vehicle_statemachine, int_data);
    return true;
}