#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "base_msgs.pb.h"
#include "ht_can_version.h"
#include "hytech_msgs_version.h"
#include "VCR_Globals.h"
#include "IOExpanderUtils.h"
#include "InverterInterface.h"
#include "MCP23017.h"
#include <algorithm>

hytech_msgs_VCRData_s VCREthernetInterface::make_vcr_data_msg(
    ADCInterface &ADCInterfaceInstance,
    DrivetrainDynamicReport_s &DrivetrainData,
    VCFHeartbeatData_s &VCF_Heartbeat_Data,
    VehicleState_e &vehicle_state_machine_state,
    DrivetrainState_e drivetrain_state_machine_state,
    veh_vec<InverterData_s> &InverterData,
    DrivebrainControllerStatus_s &DB_Controller_Status,
    TorqueControllerMuxStatus_s &tc_mux_status,
    CurrentSensorData_s &current_sensor_data)
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
    out.rear_loadcell_data.RL_loadcell_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_RL_load_cell());
    out.rear_loadcell_data.RR_loadcell_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_RR_load_cell());

    //RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_RL_sus_pot().conversion);
    out.rear_suspot_data.RR_sus_pot_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_RR_sus_pot().conversion);

    // ShutdownSensingData_s
    out.vcr_shutdown_data.i_shutdown_in = false; //shared_state.interface_data.shutdown_sensing_data.i_shutdown_in;
    out.vcr_shutdown_data.j_bspd_relay = false; //shared_state.interface_data.shutdown_sensing_data.j_bspd_relay;
    out.vcr_shutdown_data.k_watchdog_relay = false; //shared_state.interface_data.shutdown_sensing_data.k_watchdog_relay;
    out.vcr_shutdown_data.l_bms_relay = false; //shared_state.interface_data.shutdown_sensing_data.l_bms_relay;
    out.vcr_shutdown_data.m_imd_relay = false; //shared_state.interface_data.shutdown_sensing_data.m_imd_relay;

    uint16_t data = IOExpanderInstance::instance().read();

    out.vcr_shutdown_data.bspd_is_ok = IOExpanderUtils::getBit(data, 0, 1);
    out.vcr_shutdown_data.watchdog_is_ok = IOExpanderUtils::getBit(data, 1, 3);
    out.vcr_shutdown_data.bms_is_ok = IOExpanderUtils::getBit(data, 1, 1);
    out.vcr_shutdown_data.imd_is_ok = IOExpanderUtils::getBit(data, 1, 2);

    // VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = IOExpanderUtils::getBit(data, 1, 4);
    out.ethernet_is_linked.debug_link = shared_state.interface_data.ethernet_is_linked.debug_link; // fix this still
    out.ethernet_is_linked.drivebrain_link = IOExpanderUtils::getBit(data, 0, 4);
    out.ethernet_is_linked.teensy_link = IOExpanderUtils::getBit(data, 1, 5);
    out.ethernet_is_linked.ubiquiti_link = IOExpanderUtils::getBit(data, 0, 5);
    out.ethernet_is_linked.vcf_link = IOExpanderUtils::getBit(data, 1, 6);

    // veh_vec<InverterData>
    copy_inverter_data(InverterData.FL, out.inverter_data.FL);
    out.inverter_data.has_FL = true;
    copy_inverter_data(InverterData.FR, out.inverter_data.FR);
    out.inverter_data.has_FR = true;
    copy_inverter_data(InverterData.RL, out.inverter_data.RL);
    out.inverter_data.has_RL = true;
    copy_inverter_data(InverterData.RR, out.inverter_data.RR);
    out.inverter_data.has_RR = true;

    //CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = ADCInterfaceInstance.read_glv().conversion;
    out.current_sensor_data.current_sensor_unfiltered = ADCInterfaceInstance.read_bspd_current().conversion;
    out.current_sensor_data.current_refererence_unfiltered = ADCInterfaceInstance.read_bspd_reference_current().conversion;

    //DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = DrivetrainData.measuredInverterFLPackVoltage;

    copy_veh_vec_members(DrivetrainData.measuredSpeeds, out.drivetrain_data.measuredSpeeds);
    copy_veh_vec_members(DrivetrainData.measuredTorques, out.drivetrain_data.measuredTorques);
    copy_veh_vec_members(DrivetrainData.measuredTorqueCurrents, out.drivetrain_data.measuredTorqueCurrents);
    copy_veh_vec_members(DrivetrainData.measuredMagnetizingCurrents, out.drivetrain_data.measuredMagnetizingCurrents);

    //TorqueControllerMuxStatus
    out.tcmux_status.active_error = (hytech_msgs_TorqueControllerMuxError_e) tc_mux_status.active _error; // ??
    out.tcmux_status.active_controller_mode = (hytech_msgs_ControllerMode_e) tc_mux_status.active_controller_mode;
    out.tcmux_status.active_torque_limit_enum = (hytech_msgs_TorqueLimit_e) tc_mux_status.active_torque_limit_enum;
    out.tcmux_status.active_torque_limit_value = tc_mux_status.active_torque_limit_value;
    out.tcmux_status.output_is_bypassing_limits = tc_mux_status.output_is_bypassing_limits;

    // Buzzer
    out.buzzer_is_active = ADCInterfaceInstance.read_glv().conversion;

    // GLV Measurement
    out.measured_glv = current_sensor_data.twentyfour_volt_sensor;

    out.firmware_version_info.project_is_dirty = shared_state.fw_version_info.project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = shared_state.fw_version_info.project_on_main_or_master;
    std::copy(shared_state.fw_version_info.fw_version_hash.begin(), shared_state.fw_version_info.fw_version_hash.end(), out.firmware_version_info.git_hash);
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
    out.status.vehicle_state = static_cast<hytech_msgs_VehicleState_e>(vehicle_state_machine_state);
    out.status.drivetrain_state = static_cast<hytech_msgs_DrivetrainState_e>(drivetrain_state_machine_state);

    out.status.drivebrain_controller_timing_failure = DB_Controller_Status.drivebrain_controller_timing_failure;
    out.status.drivebrain_is_in_control = DB_Controller_Status.drivebrain_is_in_control;

    out.status.pedals_heartbeat_ok = VCF_Heartbeat_Data.heartbeat_ok;

    return out;
}

void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.
}

void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
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

void VCREthernetInterface::copy_inverter_data(const InverterData_s &original, hytech_msgs_InverterData_s &destination)
{
    destination.actual_motor_torque = original.actual_motor_torque;
    destination.actual_power = original.actual_power;
    destination.commanded_torque = original.commanded_torque;
    destination.dc_bus_voltage = original.dc_bus_voltage;
    destination.dc_on = original.dc_on;
    destination.derating_on = original.derating_on;
    destination.diagnostic_number = original.diagnostic_number;
    destination.error = original.error;
    destination.feedback_torque = original.feedback_torque;
    destination.igbt_temp = original.igbt_temp;
    destination.inverter_on = original.inverter_on;
    destination.inverter_temp = original.inverter_temp;
    destination.motor_temp = original.motor_temp;
    destination.quit_dc_on = original.quit_dc_on;
    destination.quit_inverter_on = original.quit_inverter_on;
    destination.speed_rpm = original.speed_rpm;
    destination.system_ready = original.system_ready;
    destination.warning = original.warning;
}