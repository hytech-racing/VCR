#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "VCR_Constants.h"
#include "base_msgs.pb.h"
#include "ht_can_version.h"
#include "hytech_msgs_version.h"
#include <algorithm>

hytech_msgs_VCRData_s VCREthernetInterface::make_vcr_data_msg(const VCRData_s &shared_state)
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
    out.has_shutdown_sensing_data = true;
    out.has_tcmux_status = true;
    out.has_msg_versions = true;
    out.has_status = true;

    out.has_vcr_loc_data = true;

    //RearLoadCellData_s
    out.rear_loadcell_data.RL_loadcell_analog = shared_state.interface_data.rear_loadcell_data.RL_loadcell_analog;
    out.rear_loadcell_data.RR_loadcell_analog = shared_state.interface_data.rear_loadcell_data.RR_loadcell_analog;

    //RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = shared_state.interface_data.rear_suspot_data.RL_sus_pot_analog;
    out.rear_suspot_data.RR_sus_pot_analog = shared_state.interface_data.rear_suspot_data.RR_sus_pot_analog;

    //ShutdownSensingData_s
    out.shutdown_sensing_data.i_shutdown_in = shared_state.interface_data.shutdown_sensing_data.i_shutdown_in;
    out.shutdown_sensing_data.j_bspd_relay = shared_state.interface_data.shutdown_sensing_data.j_bspd_relay;
    out.shutdown_sensing_data.k_watchdog_relay = shared_state.interface_data.shutdown_sensing_data.k_watchdog_relay;
    out.shutdown_sensing_data.l_bms_relay = shared_state.interface_data.shutdown_sensing_data.l_bms_relay;
    out.shutdown_sensing_data.m_imd_relay = shared_state.interface_data.shutdown_sensing_data.m_imd_relay;
    out.shutdown_sensing_data.bspd_is_ok = shared_state.interface_data.shutdown_sensing_data.bspd_is_ok;
    out.shutdown_sensing_data.watchdog_is_ok = shared_state.interface_data.shutdown_sensing_data.watchdog_is_ok;
    out.shutdown_sensing_data.bms_is_ok = shared_state.interface_data.shutdown_sensing_data.bms_is_ok;
    out.shutdown_sensing_data.imd_is_ok = shared_state.interface_data.shutdown_sensing_data.imd_is_ok;

    //VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = shared_state.interface_data.ethernet_is_linked.acu_link;
    out.ethernet_is_linked.debug_link = shared_state.interface_data.ethernet_is_linked.debug_link;
    out.ethernet_is_linked.drivebrain_link = shared_state.interface_data.ethernet_is_linked.drivebrain_link;
    out.ethernet_is_linked.teensy_link = shared_state.interface_data.ethernet_is_linked.teensy_link;
    out.ethernet_is_linked.ubiquiti_link = shared_state.interface_data.ethernet_is_linked.ubiquiti_link;
    out.ethernet_is_linked.vcf_link = shared_state.interface_data.ethernet_is_linked.vcf_link;

    //veh_vec<InverterData>

    copy_inverter_data(shared_state.interface_data.inverter_data.FL, out.inverter_data.FL);
    out.inverter_data.has_FL = true;
    copy_inverter_data(shared_state.interface_data.inverter_data.FR, out.inverter_data.FR);
    out.inverter_data.has_FR = true;
    copy_inverter_data(shared_state.interface_data.inverter_data.RL, out.inverter_data.RL);
    out.inverter_data.has_RL = true;
    copy_inverter_data(shared_state.interface_data.inverter_data.RR, out.inverter_data.RR);
    out.inverter_data.has_RR = true;

    //CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = shared_state.interface_data.current_sensor_data.twentyfour_volt_sensor;
    out.current_sensor_data.current_sensor_unfiltered = shared_state.interface_data.current_sensor_data.current_sensor_unfiltered;
    out.current_sensor_data.current_refererence_unfiltered = shared_state.interface_data.current_sensor_data.current_refererence_unfiltered;

    //DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = shared_state.system_data.drivetrain_data.measuredInverterFLPackVoltage;
    copy_veh_vec_members(shared_state.system_data.drivetrain_data.measuredSpeeds, out.drivetrain_data.measuredSpeeds);
    copy_veh_vec_members(shared_state.system_data.drivetrain_data.measuredTorques, out.drivetrain_data.measuredTorques);
    copy_veh_vec_members(shared_state.system_data.drivetrain_data.measuredTorqueCurrents, out.drivetrain_data.measuredTorqueCurrents);
    copy_veh_vec_members(shared_state.system_data.drivetrain_data.measuredMagnetizingCurrents, out.drivetrain_data.measuredMagnetizingCurrents);

    //TorqueControllerMuxStatus
    out.tcmux_status.active_error = (hytech_msgs_TorqueControllerMuxError_e) shared_state.system_data.tc_mux_status.active_error;
    out.tcmux_status.active_controller_mode = (hytech_msgs_ControllerMode_e) shared_state.system_data.tc_mux_status.active_controller_mode;
    out.tcmux_status.active_torque_limit_enum = (hytech_msgs_TorqueLimit_e) shared_state.system_data.tc_mux_status.active_torque_limit_enum;
    out.tcmux_status.active_torque_limit_value = shared_state.system_data.tc_mux_status.active_torque_limit_value;
    out.tcmux_status.output_is_bypassing_limits = shared_state.system_data.tc_mux_status.output_is_bypassing_limits;    

    // Buzzer
    out.buzzer_is_active = shared_state.system_data.buzzer_is_active;

    // GLV Measurement
    out.measured_glv = shared_state.interface_data.current_sensor_data.twentyfour_volt_sensor;
    
    out.firmware_version_info.project_is_dirty = shared_state.fw_version_info.project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = shared_state.fw_version_info.project_on_main_or_master;
    std::copy(shared_state.fw_version_info.fw_version_hash.begin(), shared_state.fw_version_info.fw_version_hash.end(), out.firmware_version_info.git_hash);
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;
    std::copy(version, version + std::min(strlen(version), sizeof(out.msg_versions.ht_proto_version) - 1), out.msg_versions.ht_proto_version);    
    out.msg_versions.ht_proto_version[sizeof(out.msg_versions.ht_proto_version) - 1] = '\0';

    // // VCR Status
    // const char* state_label = "UNKNOWN";
    out.status.vehicle_state = static_cast<hytech_msgs_VehicleState_e>(shared_state.system_data.vehicle_state_machine_state);
    out.status.drivetrain_state = static_cast<hytech_msgs_DrivetrainState_e>(shared_state.system_data.drivetrain_state_machine_state);
    
    out.status.drivebrain_controller_timing_failure = shared_state.system_data.db_cntrl_status.drivebrain_controller_timing_failure;
    out.status.drivebrain_is_in_control = shared_state.system_data.db_cntrl_status.drivebrain_is_in_control;
    
    
    
    out.status.pedals_heartbeat_ok = shared_state.system_data.vcf_heartbeat_data.heartbeat_ok;

    out.vcr_loc_data.ACU_LOC = shared_state.system_data.vcr_loc_data.acu_loc;
    out.vcr_loc_data.VCF_LOC = shared_state.system_data.vcr_loc_data.vcf_loc;

    return out;
}

void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.    
    
}

void VCREthernetInterface::receive_pb_msg_acu(const hytech_msgs_ACUAllData &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    if (!shared_state.system_data.acu_heartbeat_data.last_heartbeat_time) {
        shared_state.system_data.acu_heartbeat_data.last_heartbeat_time = curr_millis;
    } else {
        if (curr_millis - shared_state.system_data.acu_heartbeat_data.last_heartbeat_time > ACU_ACU_OK_MAX_HEARTBEAT_MS) {
            shared_state.system_data.acu_heartbeat_data.heartbeat_ok = false;
            shared_state.system_data.vcr_loc_data.acu_loc = true;
        } else {
            shared_state.system_data.acu_heartbeat_data.heartbeat_ok = true;
        }

        shared_state.interface_data.stamped_acu_core_data.acu_data.pack_voltage = msg_in.core_data.pack_voltage;
        shared_state.interface_data.stamped_acu_core_data.acu_data.min_cell_voltage = msg_in.core_data.min_cell_voltage;
        shared_state.interface_data.stamped_acu_core_data.acu_data.avg_cell_voltage = msg_in.core_data.avg_cell_voltage;
        shared_state.interface_data.stamped_acu_core_data.acu_data.max_cell_voltage = msg_in.core_data.max_cell_voltage;
        shared_state.interface_data.stamped_acu_core_data.acu_data.max_cell_temp = msg_in.core_data.max_cell_temp;
        shared_state.interface_data.stamped_acu_core_data.acu_data.max_board_temp = msg_in.core_data.max_board_temp;
        shared_state.interface_data.stamped_acu_core_data.acu_data.measured_pack_out_voltage = msg_in.core_data.pack_voltage;
        shared_state.interface_data.stamped_acu_core_data.acu_data.measured_ts_out_voltage = msg_in.core_data.max_measured_tractive_system_voltage; // should we use max or min?
        shared_state.interface_data.stamped_acu_core_data.acu_data.measured_glv = msg_in.core_data.max_measured_glv; // should we use max or min?
        
        shared_state.interface_data.stamped_acu_core_data.last_recv_millis = curr_millis;
        shared_state.system_data.acu_heartbeat_data.last_heartbeat_time = curr_millis;
    }
}

void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state, unsigned long curr_millis)
{
    // //DashInputState_s
    if (!shared_state.system_data.vcf_heartbeat_data.last_heartbeat_time) {
        shared_state.system_data.vcf_heartbeat_data.heartbeat_ok = true;
    } else {
        if (curr_millis - shared_state.system_data.vcf_heartbeat_data.last_heartbeat_time > VCF_PEDALS_MAX_HEARTBEAT_MS) {
            shared_state.system_data.vcf_heartbeat_data.heartbeat_ok = false;
            shared_state.system_data.vcr_loc_data.vcf_loc = true;
        } else {
            shared_state.system_data.vcf_heartbeat_data.heartbeat_ok = true;
        }
        shared_state.interface_data.dash_input_state.data_btn_is_pressed = msg_in.dash_input_state.data_btn_is_pressed;
        shared_state.interface_data.dash_input_state.dial_state = (ControllerMode_e) msg_in.dash_input_state.dial_state;
        shared_state.interface_data.dash_input_state.dim_btn_is_pressed = msg_in.dash_input_state.dim_btn_is_pressed;
        shared_state.interface_data.dash_input_state.left_paddle_is_pressed = msg_in.dash_input_state.left_paddle_is_pressed;
        shared_state.interface_data.dash_input_state.right_paddle_is_pressed = msg_in.dash_input_state.right_paddle_is_pressed;
        shared_state.interface_data.dash_input_state.mc_reset_btn_is_pressed = msg_in.dash_input_state.mc_reset_btn_is_pressed;
        shared_state.interface_data.dash_input_state.mode_btn_is_pressed = msg_in.dash_input_state.mode_btn_is_pressed;
        shared_state.interface_data.dash_input_state.preset_btn_is_pressed = msg_in.dash_input_state.preset_btn_is_pressed;
        shared_state.interface_data.dash_input_state.start_btn_is_pressed = msg_in.dash_input_state.start_btn_is_pressed;

    
        shared_state.system_data.vcf_heartbeat_data.last_heartbeat_time = curr_millis;
    }
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