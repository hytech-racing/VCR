#include "SystemTimeInterface.h"
#include "controllers/SimpleController.h"
#include "core_pins.h"
#include <cstdint>
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

#include "controls.h"
/* From pio-git-hash */
#include "device_fw_version.h"

/* externed CAN instances */
FlexCAN_Type<CAN3> VCRCANInterfaceImpl::TELEM_CAN;
FlexCAN_Type<CAN2> VCRCANInterfaceImpl::INVERTER_CAN;

/* Ethernet message sockets */
qindesign::network::EthernetUDP vcr_data_send_socket;
qindesign::network::EthernetUDP vcf_data_recv_socket;
qindesign::network::EthernetUDP acu_core_data_recv_socket;
qindesign::network::EthernetUDP acu_all_data_recv_socket;

/* Drivetrain Initialization */

// Inverter Interfaces
InverterInterface fl_inverter_int(INV1_CONTROL_WORD_CANID, INV1_CONTROL_INPUT_CANID, INV1_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE});
InverterInterface fr_inverter_int(INV2_CONTROL_WORD_CANID, INV2_CONTROL_INPUT_CANID, INV2_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE});
InverterInterface rl_inverter_int(INV3_CONTROL_WORD_CANID, INV3_CONTROL_INPUT_CANID, INV3_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE});
InverterInterface rr_inverter_int(INV4_CONTROL_WORD_CANID, INV4_CONTROL_INPUT_CANID, INV4_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE});

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

VCRControls controls(&drivetrain_system, MAX_ALLOWED_DB_LATENCY_MS);

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();


uint16_t state_global;
etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCRCANInterfaceImpl::vcr_CAN_recv>();

bool drivetrain_initialized = false;
TorqueControllerSimple mode0;

bool run_main_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    auto new_interface_data = sample_async_data(main_can_recv, VCRAsynchronousInterfacesInstance::instance(), vcr_data.interface_data, {
        .vcr_data_send_socket = vcr_data_send_socket,
        .vcf_data_recv_socket = vcf_data_recv_socket,
        .acu_core_data_recv_socket = acu_core_data_recv_socket,
        .acu_all_data_recv_socket = acu_all_data_recv_socket
    });
    auto sys_data = evaluate_async_systems(new_interface_data);
    auto state = VehicleStateMachineInstance::instance().tick_state_machine(sys_time::hal_millis());
    state_global = static_cast<uint16_t>(state);
    vcr_data.system_data = sys_data;
    vcr_data.interface_data = new_interface_data;

    return true;
}

/* Task Declarations */
HT_TASK::Task adc_0_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc0_task, adc0_priority, adc0_sample_period_us);
// HT_TASK::Task adc_1_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc1_task, adc1_priority, adc1_sample_period_us);
HT_TASK::Task kick_watchdog_task(init_kick_watchdog, run_kick_watchdog, watchdog_priority, kick_watchdog_period_us); 
HT_TASK::Task ams_system_task(init_ams_system_task, run_ams_system_task, ams_priority, ams_update_period_us);
HT_TASK::Task enqueue_suspension_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_suspension_CAN_data, suspension_priority, suspension_can_period_us);
HT_TASK::Task enqueue_inverter_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_inverter_CAN_data, inverter_send_priority, inv_send_period);
HT_TASK::Task send_CAN_task(HT_TASK::DUMMY_FUNCTION, handle_send_all_CAN_data, send_can_priority); // Sends all messages from the CAN queue
HT_TASK::Task vcr_data_ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, ethernet_send_priority);
HT_TASK::Task IOExpander_read_task(init_ioexpander, read_ioexpander, ioexpander_priority, ioexpander_sample_period_us);
HT_TASK::Task main_task(HT_TASK::DUMMY_FUNCTION, run_main_task, main_task_priority, main_task_period_us);
HT_TASK::Task update_brakelight_task(init_update_brakelight_task, run_update_brakelight_task, update_brakelight_priority, update_brakelight_period_us);

bool debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    Serial.println("timestamp\t:\taccel\t:\tbrake");
    Serial.print(vcr_data.interface_data.recvd_pedals_data.last_recv_millis);
    Serial.print("\t:\t");
    Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.accel_percent);
    Serial.print("\t:\t");
    Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.brake_percent);
    Serial.println();
    Serial.println("pedals heartbeat good:");
    Serial.print(vcr_data.interface_data.recvd_pedals_data.heartbeat_ok);
    Serial.println();
    Serial.println();
    Serial.println();
    Serial.println();

    Serial.println("state machine state");

    Serial.println(state_global);
    Serial.println("desired speeds, torq lim");
    Serial.println(controls._debug_dt_command.desired_speeds.FL);
    Serial.println(controls._debug_dt_command.torque_limits.FL);

    Serial.println("drivetrain system state: ");
    Serial.println(static_cast<int>(drivetrain_system.get_state()));

    Serial.print("Start button pressed: ");
    Serial.println(vcr_data.interface_data.dash_input_state.start_btn_is_pressed);
    
    Serial.print("mc reset button pressed: ");
    Serial.println(vcr_data.interface_data.dash_input_state.mc_reset_btn_is_pressed);
    return true;
}

HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, debug_print, 100, 100000); //NOLINT (priority and loop rate)

void setup() {
    // Save firmware version
    vcr_data.fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    vcr_data.fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    vcr_data.fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

    pinMode(INVERTER_ENABLE_PIN, OUTPUT);

    SPI.begin();
    
    // Create all singletons
    IOExpanderInstance::create(0);
    VCFInterfaceInstance::create(sys_time::hal_millis(), VCF_PEDALS_MAX_HEARTBEAT_MS);
    DrivebrainInterfaceInstance::create(vcr_data.interface_data.rear_loadcell_data,
        vcr_data.interface_data.rear_suspot_data,
        EthernetIPDefsInstance::instance().drivebrain_ip,
        EthernetIPDefsInstance::instance().VCRData_port,
        &vcr_data_send_socket);
    CANInterfacesInstance::create(
        VCFInterfaceInstance::instance(),
        DrivebrainInterfaceInstance::instance(), 
        fl_inverter_int,
        fr_inverter_int,
        rl_inverter_int,
        rr_inverter_int
    );
    VCRAsynchronousInterfacesInstance::create(CANInterfacesInstance::instance());

    // VehicleStateMachine vehicle_statemachine = VehicleStateMachine(
    //     etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::hv_over_threshold, drivetrain_system>(), 
    //     etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_start_button_pressed>(VCFInterfaceInstance::instance()), 
    //     etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_brake_pressed>(VCFInterfaceInstance::instance()),
    //     etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_error_present, drivetrain_system>(),
    //     etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_ready, drivetrain_system>(),
    //     etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_buzzer_start_message>(VCFInterfaceInstance::instance()),
    //     etl::delegate<void()>::create<VCRControls, &VCRControls::handle_drivetrain_command, controls>(), 
    //     etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_pedals_heartbeat_ok>(VCFInterfaceInstance::instance()),
    //     etl::delegate<void()>::create<VCFInterface, &VCFInterface::reset_pedals_heartbeat>(VCFInterfaceInstance::instance())
    // );

    VehicleStateMachineInstance::create(
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::hv_over_threshold, drivetrain_system>(), 
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_start_button_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_brake_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_error_present, drivetrain_system>(),
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_ready, drivetrain_system>(),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_buzzer_start_message>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_recalibrate_pedals_message>(VCFInterfaceInstance::instance()),
        etl::delegate<void(bool)>::create<VCRControls, &VCRControls::handle_drivetrain_command, controls>(), 
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_pedals_heartbeat_not_ok>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::reset_pedals_heartbeat>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_drivetrain_reset_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_calibrate_pedals_button_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<DrivetrainSystem, &DrivetrainSystem::reset_dt_error, drivetrain_system>()
    );

    // Scheduler timing function
    scheduler.setTimingFunction(micros);

    // Initializes all ethernet
    EthernetIPDefsInstance::create();
    uint8_t mac[6]; // NOLINT (mac addresses are always 6 bytes)
    qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(mac,
        EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().default_dns,
        EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    vcr_data_send_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
    acu_core_data_recv_socket.begin(EthernetIPDefsInstance::instance().ACUCoreData_port);
    acu_all_data_recv_socket.begin(EthernetIPDefsInstance::instance().ACUAllData_port);

    // Initialize CAN
    const uint32_t CAN_baudrate = 500000;
    handle_CAN_setup(VCRCANInterfaceImpl::INVERTER_CAN, CAN_baudrate, &VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(VCRCANInterfaceImpl::TELEM_CAN, CAN_baudrate, &VCRCANInterfaceImpl::on_telem_can_receive);

    init_adc_bundle();

    scheduler.schedule(adc_0_sample_task);
    // scheduler.schedule(adc_1_sample_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(ams_system_task);
    // scheduler.schedule(enqueue_suspension_CAN_task);
    scheduler.schedule(send_CAN_task);
    // scheduler.schedule(vcr_data_ethernet_send);
    scheduler.schedule(enqueue_inverter_CAN_task);
    scheduler.schedule(main_task);
    // scheduler.schedule(debug_state_print_task);
    scheduler.schedule(update_brakelight_task);
    
    // scheduler.schedule(IOExpander_read_task); // Commented out because i2c timeout

    // while(!Serial) {}; // hold your horses
}

void loop() {
    scheduler.run();
}