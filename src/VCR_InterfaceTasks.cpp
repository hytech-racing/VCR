#include "CANInterface.h"
#include "VCRCANInterfaceImpl.h"
#include "SystemTimeInterface.h"
#include "VCR_InterfaceTasks.h"
#include "ht_task.hpp"
#include "ACUInterface.h"
#include "ADCInterface.h"
#include "FlowmeterInterface.h"

/* From shared-systems-lib */
#include "Logger.h"

/* Local includes */
#include "VCREthernetInterface.h"
#include "VCR_Constants.h"
#include "controls.h"

#include "DrivebrainInterface.h"
#include "IOExpanderInterface.h"

#include "VCR_Globals.h" //TODO: remove

HT_TASK::TaskResponse run_read_adc0_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc0();
    ADCInterfaceInstance::instance().update_filtered_values(VCRInterfaceConstants::LOADCELL_IIR_FILTER_ALPHA);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ADCInterfaceInstance::instance().tick_adc1();
    return HT_TASK::TaskResponse::YIELD;
}

// doesn't need to exist anymore
// HT_TASK::TaskResponse run_sample_flowmeter(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
// {
//     vcr_data.interface_data.flowmeter_data.flowmeter_gallons_per_min = FlowmeterInterfaceInstance::instance().get_flow_gpm(sys_time::hal_millis());
//     return HT_TASK::TaskResponse::YIELD;
// }

HT_TASK::TaskResponse init_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUInterfaceInstance::create(sys_time::hal_millis(), VCRInterfaceConstants::ACU_ACU_OK_MAX_HEARTBEAT_MS); // NOLINT
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse update_acu_heartbeat(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    ACUCANInterfaceData_s data = ACUInterfaceInstance::instance().get_latest_data(sys_time::hal_millis());
    digitalWrite(VCRInterfaceConstants::SOFTWARE_OK_PIN, data.heartbeat_ok);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    WatchdogInstance::create(VCRInterfaceConstants::WATCHDOG_KICK_INTERVAL_MS); // NOLINT
    pinMode(VCRInterfaceConstants::SOFTWARE_OK_PIN, OUTPUT);
    pinMode(VCRInterfaceConstants::WATCHDOG_PIN, OUTPUT);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(VCRInterfaceConstants::WATCHDOG_PIN, WatchdogInstance::instance().get_watchdog_state(sys_time::hal_millis()));
    digitalWrite(VCRInterfaceConstants::SOFTWARE_OK_PIN, HIGH);
    return HT_TASK::TaskResponse::YIELD;
}

// CAN send tasks

// adds rear suspension and vcr status CAN messages to the sent on next mega loop run
HT_TASK::TaskResponse enqueue_suspension_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo )
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_suspension_CAN_data(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_controls_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRControlsInstance::instance().send_controls_can_messages();
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_coolant_temp_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_enqueue_coolant_temp_CAN_data(ADCInterfaceInstance::instance());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_flowmeter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
  DrivebrainInterfaceInstance::instance().handle_enqueue_flowmeter_CAN_data(FlowmeterInterfaceInstance::instance(), millis());
  return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_dashboard_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFInterfaceInstance::instance().enqueue_vehicle_state_message(VehicleStateMachineInstance::instance().get_state(),
                                                                DrivetrainInstance::instance().get_state(),
                                                                VCRControlsInstance::instance().drivebrain_is_in_control());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enqueue_inverter_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    CANInterfacesInstance::instance().fl_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().fl_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().fr_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().fr_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().rl_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().rl_inverter_interface.send_INV_SETPOINT_COMMAND();

    CANInterfacesInstance::instance().rr_inverter_interface.send_INV_CONTROL_WORD();
    CANInterfacesInstance::instance().rr_inverter_interface.send_INV_SETPOINT_COMMAND();

    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_all_CAN_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstace::instance().inverter_can_tx_buffer, &VCRCANInterfaceInstace::instance().INVERTER_CAN);
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstace::instance().telem_can_tx_buffer, &VCRCANInterfaceInstace::instance().TELEM_CAN);
    VCRCANInterfaceImpl::send_all_CAN_msgs(VCRCANInterfaceInstace::instance().rear_aux_can_tx_buffer, &VCRCANInterfaceInstace::instance().REAR_AUX_CAN);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse handle_send_VCR_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    DrivebrainInterfaceInstance::instance().handle_send_ethernet_data(
        VCREthernetInterface::makeVCRDataMsg(ADCInterfaceInstance::instance(),
        vcr_data.system_data.drivetrain_data, VCFInterfaceInstance::instance(),
        VehicleStateMachineInstance::instance(), DrivetrainInstance::instance(),
        fl_inverter_int, fr_inverter_int, rl_inverter_int, rr_inverter_int,
        VCRControlsInstance::instance()));
    return HT_TASK::TaskResponse::YIELD;
}

// TODO: replace these with a separate interface (should likely just create general IOexpander interface and then keep logic in here based on specific values)
// need to double check pin assigments
HT_TASK::TaskResponse read_ioexpander(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) // TODO: make all of this in a separate IO Expander Interface
{
    IOExpanderInterfaceInstance::instance().read();

    // inputs on port a (0)
    vcr_data.interface_data.shutdown_sensing_data.bspd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortA(0);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(1);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_fault = IOExpanderInterfaceInstance::instance().getBitPortA(2);
    vcr_data.interface_data.ethernet_is_linked.vn_link = IOExpanderInterfaceInstance::instance().getBitPortA(3);
    vcr_data.interface_data.ethernet_is_linked.drivebrain_link = IOExpanderInterfaceInstance::instance().getBitPortA(4);
    vcr_data.interface_data.ethernet_is_linked.ubiquiti_link = IOExpanderInterfaceInstance::instance().getBitPortA(5);
    // vcr_data.interface_data.shutdown_sensing_data.bspd_missing = IOExpanderInterfaceInstance::instance().getBitPortA(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortA(7);

    // inputs on port b (1)
    // vcr_data.interface_data.shutdown_sensing_data.lv_present = IOExpanderInterfaceInstance::instance().getBitPortB(0);
    vcr_data.interface_data.shutdown_sensing_data.bms_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(1);
    vcr_data.interface_data.shutdown_sensing_data.imd_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(2);
    vcr_data.interface_data.shutdown_sensing_data.vcr_sw_is_ok = IOExpanderInterfaceInstance::instance().getBitPortB(3);
    vcr_data.interface_data.ethernet_is_linked.acu_link = IOExpanderInterfaceInstance::instance().getBitPortB(4);
    vcr_data.interface_data.ethernet_is_linked.teensy_link = IOExpanderInterfaceInstance::instance().getBitPortB(5);
    vcr_data.interface_data.ethernet_is_linked.vcf_link = IOExpanderInterfaceInstance::instance().getBitPortB(6);
    // nothing = IOExpanderInterfaceInstance::instance().getBitPortB(7);
 
    return HT_TASK::TaskResponse::YIELD;
    // NOLINTEND
}

HT_TASK::TaskResponse init_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    pinMode(VCRInterfaceConstants::BRAKELIGHT_CONTROL_PIN, OUTPUT);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse run_update_brakelight_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    digitalWrite(VCRInterfaceConstants::BRAKELIGHT_CONTROL_PIN, VCFInterfaceInstance::instance().is_brake_pressed());
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enable_motor_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) 
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so is ok
    bool enable_state = vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_2 || 
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_3;
    digitalWrite(VCRInterfaceConstants::MOTOR_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);
    return HT_TASK::TaskResponse::YIELD;
}

HT_TASK::TaskResponse enable_inverter_cooling(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo) 
{
    VehicleState_e vehicle_state = VehicleStateMachineInstance::instance().get_state(); //NOLINT will alway be populated so is ok
    bool enable_state = vehicle_state == VehicleState_e::TRACTIVE_SYSTEM_ACTIVE || 
                        vehicle_state == VehicleState_e::WANTING_READY_TO_DRIVE || 
                        vehicle_state == VehicleState_e::READY_TO_DRIVE ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_2 ||
                        VCFInterfaceInstance::instance().get_latest_data().dash_input_state.dial_state == ControllerMode_e::MODE_5;
    digitalWrite(VCRInterfaceConstants::INVERTER_COOLING_CONTROL_PIN, enable_state ? HIGH : LOW);
    
    return HT_TASK::TaskResponse::YIELD;
}