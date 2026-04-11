#include "VCREthernetInterface.h"
#include "MCP23017.h"
#include "SharedFirmwareTypes.h"
#include "base_msgs.pb.h"
#include "ht_can_version.h"
#include "hytech_msgs_version.h"
#include "VCR_Globals.h"
#include "IOExpanderUtils.h"
#include "VCFInterface.h"
#include "VehicleStateMachine.h"
#include "DrivetrainSystem.h"
#include "controllers/DrivebrainController.h"
#include "controls.h"
#include <algorithm>

hytech_msgs_VCRData_s VCREthernetInterface::makeVCRDataMsg(
    const ADCInterface &adc_interface,
    DrivetrainDynamicReport_s &DrivetrainData,
    const VCFInterface &vcf_interface,
    const VehicleStateMachine &vehicle_state_machine,
    const DrivetrainSystem &drivetrain_system,
    const InverterInterface &fl_inverter,
    const InverterInterface &fr_inverter,
    const InverterInterface &rl_inverter,
    const InverterInterface &rr_inverter,
    const VCRControls &vcr_controls)
{
	hytech_msgs_VCRData_s out;

    //has_data
    out.has_current_sensor_data = true;
    out.has_drivetrain_data = true;

    out.drivetrain_data.has_measuredMagnetizingCurrents = true;
    out.drivetrain_data.has_measuredSpeeds = true;
    out.drivetrain_data.has_measuredTorqueCurrents = true;
    out.drivetrain_data.has_measuredTorques = true;

    out.has_ethernet_is_linked = true;
    out.has_firmware_version_info = true;
    out.has_rear_loadcell_data = true;
    out.has_rear_suspot_data = true;
    out.has_vcr_shutdown_data = true;
    out.has_tcmux_status = true;
    out.has_msg_versions = true;
    out.has_status = true;

    //RearLoadCellData_s
    out.rear_loadcell_data.RL_loadcell_analog = static_cast<uint32_t>(adc_interface.get_filtered_RL_load_cell());
    out.rear_loadcell_data.RR_loadcell_analog = static_cast<uint32_t>(adc_interface.get_filtered_RR_load_cell());

    //RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = static_cast<uint32_t>(adc_interface.get_RL_sus_pot().conversion);
    out.rear_suspot_data.RR_sus_pot_analog = static_cast<uint32_t>(adc_interface.get_RR_sus_pot().conversion);

    // ShutdownSensingData_s
    out.vcr_shutdown_data.i_shutdown_in = false; //shared_state.interface_data.shutdown_sensing_data.i_shutdown_in;
    out.vcr_shutdown_data.j_bspd_relay = false; //shared_state.interface_data.shutdown_sensing_data.j_bspd_relay;
    out.vcr_shutdown_data.k_watchdog_relay = false; //shared_state.interface_data.shutdown_sensing_data.k_watchdog_relay;
    out.vcr_shutdown_data.l_bms_relay = false; //shared_state.interface_data.shutdown_sensing_data.l_bms_relay;
    out.vcr_shutdown_data.m_imd_relay = false; //shared_state.interface_data.shutdown_sensing_data.m_imd_relay;

    uint16_t data = IOExpanderInstance::instance().read();

    out.vcr_shutdown_data.bspd_is_ok = IOExpanderUtils::getBit(data, 0, 1); //TODO: change numbers here to constants for readability
    out.vcr_shutdown_data.watchdog_is_ok = IOExpanderUtils::getBit(data, 1, 3);
    out.vcr_shutdown_data.bms_is_ok = IOExpanderUtils::getBit(data, 1, 1);
    out.vcr_shutdown_data.imd_is_ok = IOExpanderUtils::getBit(data, 1, 2);

    // VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = IOExpanderUtils::getBit(data, 1, 4);
    // out.ethernet_is_linked.debug_link = shared_state.interface_data.ethernet_is_linked.debug_link; // TODO: fix this still
    out.ethernet_is_linked.drivebrain_link = IOExpanderUtils::getBit(data, 0, 4);
    out.ethernet_is_linked.teensy_link = IOExpanderUtils::getBit(data, 1, 5);
    out.ethernet_is_linked.ubiquiti_link = IOExpanderUtils::getBit(data, 0, 5);
    out.ethernet_is_linked.vcf_link = IOExpanderUtils::getBit(data, 1, 6);

    // veh_vec<InverterData>
    copyInverterData(fl_inverter.get_all_inverter_data(), out.inverter_data.FL);
    out.inverter_data.has_FL = true;
    copyInverterData(fr_inverter.get_all_inverter_data(), out.inverter_data.FR);
    out.inverter_data.has_FR = true;
    copyInverterData(rl_inverter.get_all_inverter_data(), out.inverter_data.RL);
    out.inverter_data.has_RL = true;
    copyInverterData(rr_inverter.get_all_inverter_data(), out.inverter_data.RR);
    out.inverter_data.has_RR = true;

    //CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = adc_interface.get_glv().conversion;
    out.current_sensor_data.current_sensor_unfiltered = adc_interface.get_bspd_current().conversion;
    out.current_sensor_data.current_refererence_unfiltered = adc_interface.get_bspd_reference_current().conversion;
    out.current_sensor_data.bpsd_brake_high_sense = adc_interface.is_brake_sense_high();
    out.current_sensor_data.bspd_current_high_sense = adc_interface.is_current_sense_high();

    //DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = DrivetrainData.measuredInverterFLPackVoltage;

    copyVehVecMembers(DrivetrainData.measuredSpeeds, out.drivetrain_data.measuredSpeeds);
    copyVehVecMembers(DrivetrainData.measuredTorques, out.drivetrain_data.measuredTorques);
    copyVehVecMembers(DrivetrainData.measuredTorqueCurrents, out.drivetrain_data.measuredTorqueCurrents);
    copyVehVecMembers(DrivetrainData.measuredMagnetizingCurrents, out.drivetrain_data.measuredMagnetizingCurrents);

    //TorqueControllerMuxStatus
    out.tcmux_status.active_error = (hytech_msgs_TorqueControllerMuxError_e) vcr_controls.get_tc_mux_status().active_error;
    out.tcmux_status.active_controller_mode = (hytech_msgs_ControllerMode_e) vcr_controls.get_tc_mux_status().active_controller_mode;
    out.tcmux_status.active_torque_limit_enum = (hytech_msgs_TorqueLimit_e) vcr_controls.get_tc_mux_status().active_torque_limit_enum;
    out.tcmux_status.active_torque_limit_value = vcr_controls.get_tc_mux_status().active_torque_limit_value;
    out.tcmux_status.output_is_bypassing_limits = vcr_controls.get_tc_mux_status().output_is_bypassing_limits;

    // Buzzer
    out.buzzer_is_active = adc_interface.get_glv().conversion;

    // out.firmware_version_info.project_is_dirty = shared_state.fw_version_info.project_is_dirty;
    // out.firmware_version_info.project_on_main_or_master = shared_state.fw_version_info.project_on_main_or_master;
    // std::copy(shared_state.fw_version_info.fw_version_hash.begin(), shared_state.fw_version_info.fw_version_hash.end(), out.firmware_version_info.git_hash);
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;

    // working with bytes in nanopb
    std::string_view version_view(version);
    const size_t version_len = [&]() -> size_t {
        return std::min(version_view.size(), sizeof(out.msg_versions.ht_proto_version.bytes));
    }();
    out.msg_versions.ht_proto_version.size = version_len;
    std::copy(version_view.begin(), version_view.begin() + version_len, std::begin(out.msg_versions.ht_proto_version.bytes));

    // // VCR Status
    // const char* state_label = "UNKNOWN";
    out.status.vehicle_state = static_cast<hytech_msgs_VehicleState_e>(vehicle_state_machine.get_state());
    out.status.drivetrain_state = static_cast<hytech_msgs_DrivetrainState_e>(drivetrain_system.get_state());

    out.status.drivebrain_controller_timing_failure = vcr_controls.drivebrain_timing_failure();
    out.status.drivebrain_is_in_control = vcr_controls.drivebrain_is_in_control();

    out.status.pedals_heartbeat_ok = vcf_interface.get_latest_data().stamped_pedals.heartbeat_ok;

    return out;
}

void VCREthernetInterface::receivePbMsgDB(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.
}

void VCREthernetInterface::receivePbMsgVCF(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    // //DashInputState_s
    // shared_state.interface_data.dash_input_state.data_btn_is_pressed = msg_in.dash_input_state.data_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.dial_state = (ControllerMode_e) msg_in.dash_input_state.dial_state;
    // shared_state.interface_data.dash_input_state.dim_btn_is_pressed = msg_in.dash_input_state.dim_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.left_paddle_is_pressed = msg_in.dash_input_state.left_paddle_is_pressed;
    // shared_state.interface_data.dash_input_state.right_paddle_is_pressed = msg_in.dash_input_state.right_paddle_is_pressed;
    // shared_state.interface_data.dash_input_state.mc_reset_btn_is_pressed = msg_in.dash_input_state.mc_reset_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.mode_btn_is_pressed = msg_in.dash_input_state.mode_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.preset_btn_is_pressed = msg_in.dash_input_state.preset_btn_is_pressed;
    // shared_state.interface_data.dash_input_state.start_btn_is_pressed = msg_in.dash_input_state.start_btn_is_pressed;
}

void VCREthernetInterface::copyInverterData(const InverterFeedbackData_s &original, hytech_msgs_InverterData_s &destination)
{
    // Status
    destination.system_ready = original.status.system_ready;
    destination.error = original.status.error;
    destination.warning = original.status.warning;
    destination.quit_dc_on = original.status.quit_dc_on;
    destination.dc_on = original.status.dc_on;
    destination.quit_inverter_on = original.status.quit_inverter_on;
    destination.inverter_on = original.status.inverter_on;
    destination.derating_on = original.status.derating_on;
    destination.dc_bus_voltage = original.status.dc_bus_voltage;
    destination.diagnostic_number = original.status.diagnostic_number;

    // Temps
    destination.igbt_temp = original.temps.igbt_temp;
    destination.inverter_temp = original.temps.inverter_temp;
    destination.motor_temp = original.temps.motor_temp;

    // Power

    // Motor Mechanics
    destination.actual_power = original.motor_mechanics.actual_power;
    destination.actual_motor_torque = original.motor_mechanics.actual_torque;
    destination.speed_rpm = original.motor_mechanics.actual_speed;

    // Control Feedback
    destination.commanded_torque = 0; //TODO: figure out if this is actually used / updated
    destination.feedback_torque = 0; //TODO: figure out if this is actually used / updated
}