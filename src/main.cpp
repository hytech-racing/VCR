#ifdef ARDUINO

#include <Arduino.h>
#endif

/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

/* Arduino specific upstream Libraries */
#include "QNEthernet.h"
#include "FlexCAN_T4.h"

/* From Embedded Template Library libdep */
#include "etl/singleton.h"

/* From HT_SCHED libdep*/
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* From shared-firmware-interfaces libdep */
#include "EthernetAddressDefs.h"

/* Local includes */
#include "TorqueControllerMux.hpp"
#include "VCFInterface.h"
#include "VCREthernetInterface.h"
#include "VCR_Constants.h"
#include "VCR_Globals.h"
#include "VCR_InterfaceTasks.h"
#include "VCRCANInterfaceImpl.h"
#include "DrivebrainInterface.h"
#include "InverterInterface.h"
#include "DrivetrainSystem.h"
#include "VCR_SystemTasks.h"
#include "VehicleStateMachine.h"

/* From pio-git-hash */
#include "device_fw_version.h"

/* CAN setup */
FlexCAN_Type<CAN3> TELEM_CAN;
FlexCAN_Type<CAN2> INVERTER_CAN;

/* Ethernet message sockets */
qindesign::network::EthernetUDP vcr_data_send_socket;
qindesign::network::EthernetUDP vcf_data_recv_socket;
qindesign::network::EthernetUDP acu_core_data_recv_socket;
qindesign::network::EthernetUDP acu_all_data_recv_socket;
qindesign::network::EthernetUDP db_data_recv_socket;

/* Drivetrain Initialization */
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

DrivetrainSystem drivetrain_system(inverter_functs);

etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

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

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

constexpr unsigned long adc0_sample_period_us = 250;                 // 250 us = 4 kHz
constexpr unsigned long adc1_sample_period_us = 10000;               // 10 000 us = 100 Hz
constexpr unsigned long update_buzzer_controller_period_us = 100000; // 100 000 us = 10 Hz
constexpr unsigned long kick_watchdog_period_us = 10000;             // 10 000 us = 100 Hz
constexpr unsigned long ams_update_period_us = 10000;                // 10 000 us = 100 Hz
constexpr unsigned long ethernet_update_period = 10000;              // 10 000 us = 100 Hz
constexpr unsigned long suspension_can_period_us = 4000;             // 4 000 us = 250 Hz
constexpr unsigned long inv_send_period = 4000;                      // 4 000 us = 250 Hz
constexpr unsigned long ioexpander_sample_period_us = 50000;         // 50 000 us = 20 Hz

bool run_main_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    auto new_interface_data = sample_async_data(main_can_recv, VCRAsynchronousInterfacesInstance::instance(), vcr_data.interface_data);
    auto sys_data = evaluate_async_systems(new_interface_data);
    auto state = vehicle_statemachine.tick_state_machine(sys_time::hal_millis());

    vcr_data.system_data = sys_data;
    vcr_data.interface_data = new_interface_data;

    return true;
}

/* Task Declarations */
HT_TASK::Task adc_0_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc0_task, 5, adc0_sample_period_us);
HT_TASK::Task adc_1_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc1_task, 50, adc1_sample_period_us);
HT_TASK::Task update_buzzer_controller_task(HT_TASK::DUMMY_FUNCTION, run_update_buzzer_controller_task, 3, update_buzzer_controller_period_us);
HT_TASK::Task kick_watchdog_task(init_kick_watchdog, run_kick_watchdog, 0, kick_watchdog_period_us); 
HT_TASK::Task ams_system_task(init_ams_system_task, run_ams_system_task, 2, ams_update_period_us);
HT_TASK::Task enqueue_suspension_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_suspension_CAN_data, 4, suspension_can_period_us);
HT_TASK::Task enqueue_inverter_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_inverter_CAN_data, 5, inv_send_period);
HT_TASK::Task send_CAN_task(HT_TASK::DUMMY_FUNCTION, handle_send_all_CAN_data, 3); // Sends all messages from the CAN queue
HT_TASK::Task vcr_data_ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, 6);
HT_TASK::Task IOExpander_read_task(init_ioexpander, read_ioexpander, 100, ioexpander_sample_period_us);
HT_TASK::Task main_task(HT_TASK::DUMMY_FUNCTION, run_main_task, 0);

void setup() {

    // Save firmware version
    vcr_data.fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    vcr_data.fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    vcr_data.fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

    // Create all singletons
    IOExpanderInstance::create(0);
    VCFInterfaceInstance::create();
    CANInterfacesInstance::create(
        VCFInterfaceInstance::instance(),
        DrivebrainInterfaceInstance::instance(), 
        fl_inverter_int,
        fr_inverter_int,
        rl_inverter_int,
        rr_inverter_int
    );
    VCRAsynchronousInterfacesInstance::create(CANInterfacesInstance::instance());

    // Scheduler timing function
    scheduler.setTimingFunction(micros);

    // Initializes all ethernet
    EthernetIPDefsInstance::create();
    uint8_t mac[6];
    qindesign::network::Ethernet.macAddress(mac);
    qindesign::network::Ethernet.begin(
        EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().default_dns,
        EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    vcr_data_send_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
    acu_core_data_recv_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    acu_all_data_recv_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);
    db_data_recv_socket.begin(EthernetIPDefsInstance::instance().DBData_port);

    DrivebrainInterfaceInstance::create(vcr_data.interface_data.rear_loadcell_data,
                                        vcr_data.interface_data.rear_suspot_data,
                                        EthernetIPDefsInstance::instance().drivebrain_ip,
                                        EthernetIPDefsInstance::instance().VCRData_port,
                                        &vcr_data_send_socket);
        
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
    scheduler.schedule(enqueue_suspension_CAN_task);
    scheduler.schedule(send_CAN_task);
    scheduler.schedule(vcr_data_ethernet_send);
    scheduler.schedule(enqueue_inverter_CAN_task);
    scheduler.schedule(main_task);
    // scheduler.schedule(IOExpander_read_task); // Commented out because i2c timeout
}

void loop() { 
    scheduler.run();
}