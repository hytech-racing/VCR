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

/* Drivetrain Initialization */

// Inverter Interfaces
InverterInterface fl_inverter_int(INV1_CONTROL_WORD_CANID, INV1_CONTROL_INPUT_CANID, INV1_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface fr_inverter_int(INV2_CONTROL_WORD_CANID, INV2_CONTROL_INPUT_CANID, INV2_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface rl_inverter_int(INV3_CONTROL_WORD_CANID, INV3_CONTROL_INPUT_CANID, INV3_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT
InverterInterface rr_inverter_int(INV4_CONTROL_WORD_CANID, INV4_CONTROL_INPUT_CANID, INV4_CONTROL_PARAMETER_CANID, {.MINIMUM_HV_VOLTAGE = INVERTER_MINIMUM_HV_VOLTAGE}); //NOLINT

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

etl::delegate<void(bool)> set_ef_pin_active = etl::delegate<void(bool)>::create([](bool set_active) { digitalWrite(INVERTER_ENABLE_PIN, static_cast<int>(set_active)); });

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

/* Task Declarations */
HT_TASK::Task adc_0_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc0_task, adc0_priority, adc0_sample_period_us);
HT_TASK::Task adc_1_sample_task(HT_TASK::DUMMY_FUNCTION, run_read_adc1_task, adc1_priority, adc1_sample_period_us); //using adc1 for the temp sensors in place of the thermistors
HT_TASK::Task kick_watchdog_task(init_kick_watchdog, run_kick_watchdog, watchdog_priority, kick_watchdog_period_us); 
HT_TASK::Task ams_system_task(init_acu_heartbeat, update_acu_heartbeat, ams_priority, ams_update_period_us);
HT_TASK::Task enqueue_suspension_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_suspension_CAN_data, suspension_priority, suspension_can_period_us);
HT_TASK::Task enqueue_inverter_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_inverter_CAN_data, inverter_send_priority, inv_send_period);
HT_TASK::Task enqueue_dashboard_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_dashboard_CAN_data, dashboard_send_priority, dashboard_send_period_us);
HT_TASK::Task enqueue_coolant_temp_CAN_task(HT_TASK::DUMMY_FUNCTION, enqueue_coolant_temp_CAN_data, coolant_temp_send_priority, coolant_temp_send_period_us);
HT_TASK::Task send_CAN_task(HT_TASK::DUMMY_FUNCTION, handle_send_all_CAN_data, send_can_priority, send_can_period_us); // Sends all messages from the CAN queue
HT_TASK::Task vcr_data_ethernet_send(HT_TASK::DUMMY_FUNCTION, handle_send_VCR_ethernet_data, ethernet_send_priority, ethernet_update_period);
HT_TASK::Task IOExpander_read_task(init_ioexpander, read_ioexpander, ioexpander_priority, ioexpander_sample_period_us);
HT_TASK::Task async_main_task(HT_TASK::DUMMY_FUNCTION, run_async_main_task, main_task_priority, main_task_period_us);
HT_TASK::Task update_brakelight_task(init_update_brakelight_task, run_update_brakelight_task, update_brakelight_priority, update_brakelight_period_us);



HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Serial.println("timestamp\t:\taccel\t:\tbrake");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.last_recv_millis);
    // Serial.print("\t:\t");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.accel_percent);
    // Serial.print("\t:\t");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.pedals_data.brake_percent);
    // Serial.println();
    // Serial.println("pedals heartbeat good:");
    // Serial.print(vcr_data.interface_data.recvd_pedals_data.heartbeat_ok);
    // Serial.println();
    // Serial.println();
    // Serial.println();
    // Serial.println();

    // Serial.println("state machine state");

    // Serial.println(vcr_data.system_data.vehicle_state_machine_state);
    // Serial.println("desired speeds, torq lim");
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.desired_speeds.FL);
    // Serial.println(VCRControlsInstance::instance()._debug_dt_command.torque_limits.FL);

    // Serial.print("Drivetrain system state: ");
    // Serial.println(static_cast<int>(DrivetrainInstance::instance().get_state()));
    // Serial.print("Diagnostic FL #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.FL.diagnostic_number);
    // Serial.print(" FR #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.FR.diagnostic_number);
    // Serial.print(" RL #: ");
    // Serial.print(DrivetrainInstance::instance().get_status().inverter_statuses.RL.diagnostic_number);
    // Serial.print(" RR #: ");
    // Serial.println(DrivetrainInstance::instance().get_status().inverter_statuses.RR.diagnostic_number);

    // Serial.print("Vehicle statemachine state: ");
    // Serial.println(static_cast<int>(VehicleStateMachineInstance::instance().get_state()));

    // Serial.print("launch controller state: ");
    // Serial.println(static_cast<int>(VCRControlsInstance::instance().get_launch_controller().get_launch_state()));

    // Serial.print("Start button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.start_btn_is_pressed);

    // Serial.print("pedal recalibrate button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.preset_btn_is_pressed);
    
    // Serial.print("mc reset button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.mc_reset_btn_is_pressed);
    
    // Serial.print("torque mode cycle button pressed: ");
    // Serial.println(vcr_data.interface_data.dash_input_state.mode_btn_is_pressed);

    // Serial.println("IOExpander testing");
    // Serial.println("Shutdown Data");
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.k_watchdog_relay);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.watchdog_is_ok);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.l_bms_relay);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.bms_is_ok);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.m_imd_relay);
    // Serial.println(vcr_data.interface_data.shutdown_sensing_data.imd_is_ok);
    // Serial.println("Linked Data");
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.acu_link);
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.drivebrain_link);
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.vcf_link);
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.teensy_link);
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.debug_link);
    // Serial.println(vcr_data.interface_data.ethernet_is_linked.ubiquiti_link);


    // Serial.print("Load Cell RR: ");
    // Serial.println(vcr_data.interface_data.rear_loadcell_data.RR_loadcell_analog);

    // Serial.print("Load Cell RL: ");
    // Serial.println(vcr_data.interface_data.rear_loadcell_data.RL_loadcell_analog);

    // Serial.print("SusPot RR: ");
    // Serial.println(vcr_data.interface_data.rear_suspot_data.RR_sus_pot_analog);

    // Serial.print("SusPot RL: ");
    // Serial.println(vcr_data.interface_data.rear_suspot_data.RL_sus_pot_analog);

    /* Drivebrain data */
    // Serial.print("Latest Drivebrain data: ");
    // Serial.print(vcr_data.interface_data.latest_drivebrain_command.torque_limits.veh_vec_data.FL);
    // Serial.print(" ");
    // Serial.print(vcr_data.interface_data.latest_drivebrain_command.torque_limits.veh_vec_data.FR);
    // Serial.print(" ");
    // Serial.print(vcr_data.interface_data.latest_drivebrain_command.torque_limits.veh_vec_data.RL);
    // Serial.print(" ");
    // Serial.println(vcr_data.interface_data.latest_drivebrain_command.torque_limits.veh_vec_data.FL);
    
    /* Thermistor Data */
    Serial.print("Thermistor 0 Analog: ");
    Serial.print(vcr_data.interface_data.thermistor_data.thermistor_0.thermistor_analog);
    Serial.print(" Thermistor 0 degrees C: ");
    Serial.println(vcr_data.interface_data.thermistor_data.thermistor_0.thermistor_degrees_C);
    Serial.print("Thermistor 1 Analog: ");
    Serial.print(vcr_data.interface_data.thermistor_data.thermistor_1.thermistor_analog);
    Serial.print(" Thermistor 1 degrees C: ");
    Serial.println(vcr_data.interface_data.thermistor_data.thermistor_1.thermistor_degrees_C);

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, debug_print, 100, 100000); //NOLINT (priority and loop rate)

void setup() {
    // Save firmware version
    vcr_data.fw_version_info.fw_version_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    vcr_data.fw_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    vcr_data.fw_version_info.project_is_dirty = device_status_t::project_is_dirty;

    SPI.begin();

    pinMode(INVERTER_ENABLE_PIN, OUTPUT);
    
    // Create all singletons
    // IOExpanderInstance::create(0);
    ProtobufSocketsInstance::create(vcr_data_send_socket, vcf_data_recv_socket);
    EthernetIPDefsInstance::create();
    VCFInterfaceInstance::create(sys_time::hal_millis(), VCF_PEDALS_MAX_HEARTBEAT_MS);
    DrivebrainInterfaceInstance::create(vcr_data.interface_data.rear_loadcell_data,
        vcr_data.interface_data.rear_suspot_data,
        vcr_data.interface_data.thermistor_data.thermistor_0,
        vcr_data.interface_data.thermistor_data.thermistor_1,
        EthernetIPDefsInstance::instance().drivebrain_ip,
        EthernetIPDefsInstance::instance().VCRData_port,
        &vcr_data_send_socket);
    DrivetrainInstance::create(inverter_functs, set_ef_pin_active);

    // Initializes all ethernet
    // uint8_t mac[6]; // NOLINT (mac addresses are always 6 bytes)
    // qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().car_subnet, EthernetIPDefsInstance::instance().default_gateway);
    vcr_data_send_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_recv_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    CANInterfacesInstance::create(
        VCFInterfaceInstance::instance(),
        ACUInterfaceInstance::instance(),
        DrivebrainInterfaceInstance::instance(), 
        fl_inverter_int,
        fr_inverter_int,
        rl_inverter_int,
        rr_inverter_int
    );
    VCRAsynchronousInterfacesInstance::create(CANInterfacesInstance::instance());

    VCRControlsInstance::create(&DrivetrainInstance::instance(), MAX_ALLOWED_DB_LATENCY_MS);
    VehicleStateMachineInstance::create(
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::hv_over_threshold>(DrivetrainInstance::instance()), 
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_start_button_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_brake_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_error_present>(DrivetrainInstance::instance()),
        etl::delegate<bool()>::create<DrivetrainSystem, &DrivetrainSystem::drivetrain_ready>(DrivetrainInstance::instance()),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_buzzer_start_message>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::send_recalibrate_pedals_message>(VCFInterfaceInstance::instance()),
        etl::delegate<void(bool, bool)>::create<VCRControls, &VCRControls::handle_drivetrain_command>(VCRControlsInstance::instance()), 
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_pedals_heartbeat_not_ok>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<VCFInterface, &VCFInterface::reset_pedals_heartbeat>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_drivetrain_reset_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<bool()>::create<VCFInterface, &VCFInterface::is_recalibrate_pedals_button_pressed>(VCFInterfaceInstance::instance()),
        etl::delegate<void()>::create<DrivetrainSystem, &DrivetrainSystem::reset_dt_error>(DrivetrainInstance::instance())
    );

    // Scheduler timing function
    scheduler.setTimingFunction(micros);

    // Initialize CAN
    const uint32_t telem_CAN_baudrate = 1000000;
    const uint32_t inv_CAN_baudrate = 500000;
   
    handle_CAN_setup(VCRCANInterfaceImpl::INVERTER_CAN, inv_CAN_baudrate, &VCRCANInterfaceImpl::on_inverter_can_receive);
    handle_CAN_setup(VCRCANInterfaceImpl::TELEM_CAN, telem_CAN_baudrate, &VCRCANInterfaceImpl::on_telem_can_receive);

    init_adc_bundle();

    scheduler.schedule(adc_0_sample_task);
    scheduler.schedule(adc_1_sample_task);
    scheduler.schedule(kick_watchdog_task);
    scheduler.schedule(ams_system_task);
    scheduler.schedule(enqueue_suspension_CAN_task);
    scheduler.schedule(send_CAN_task);
    scheduler.schedule(vcr_data_ethernet_send);
    scheduler.schedule(enqueue_inverter_CAN_task);
    scheduler.schedule(enqueue_coolant_temp_CAN_task);
    scheduler.schedule(async_main_task);
    scheduler.schedule(debug_state_print_task);
    scheduler.schedule(update_brakelight_task);
    
    scheduler.schedule(IOExpander_read_task);

}

void loop() {
    scheduler.run();
}