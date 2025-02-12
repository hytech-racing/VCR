#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include <Arduino.h>

hytech_msgs_VCRData_s VCREthernetInterface::make_vcr_data_msg(VCRData_s &shared_state)
{
	hytech_msgs_VCRData_s out;

    //RearLoadCellData_s
    out.rear_loadcell_data.RL_loadcell_analog = shared_state.rear_loadcell_data.RL_loadcell_analog;
    out.rear_loadcell_data.RR_loadcell_analog = shared_state.rear_loadcell_data.RR_loadcell_analog;

    //RearSusPotData_s
    out.rear_suspot_data.RL_sus_pot_analog = shared_state.rear_suspot_data.RL_sus_pot_analog;
    out.rear_suspot_data.RR_sus_pot_analog = shared_state.rear_suspot_data.RR_sus_pot_analog;

    //ShutdownSensingData_s
    out.shutdown_sensing_data.i_shutdown_in = shared_state.shutdown_sensing_data.i_shutdown_in;
    out.shutdown_sensing_data.j_bspd_relay = shared_state.shutdown_sensing_data.j_bspd_relay;
    out.shutdown_sensing_data.k_watchdog_relay = shared_state.shutdown_sensing_data.k_watchdog_relay;
    out.shutdown_sensing_data.l_bms_relay = shared_state.shutdown_sensing_data.l_bms_relay;
    out.shutdown_sensing_data.m_imd_relay = shared_state.shutdown_sensing_data.m_imd_relay;
    out.shutdown_sensing_data.bspd_is_ok = shared_state.shutdown_sensing_data.bspd_is_ok;
    out.shutdown_sensing_data.watchdog_is_ok = shared_state.shutdown_sensing_data.watchdog_is_ok;
    out.shutdown_sensing_data.bms_is_ok = shared_state.shutdown_sensing_data.bms_is_ok;
    out.shutdown_sensing_data.imd_is_ok = shared_state.shutdown_sensing_data.imd_is_ok;

    //VCREthernetLinkData_s
    out.ethernet_is_linked.acu_link = shared_state.ethernet_is_linked.acu_link;
    out.ethernet_is_linked.debug_link = shared_state.ethernet_is_linked.debug_link;
    out.ethernet_is_linked.drivebrain_link = shared_state.ethernet_is_linked.drivebrain_link;
    out.ethernet_is_linked.teensy_link = shared_state.ethernet_is_linked.teensy_link;
    out.ethernet_is_linked.ubiquiti_link = shared_state.ethernet_is_linked.ubiquiti_link;
    out.ethernet_is_linked.vcf_link = shared_state.ethernet_is_linked.vcf_link;

    //veh_vec<InverterData>
    copy_inverter_data(shared_state.inverter_data.FL, out.inverter_data.FL);
    copy_inverter_data(shared_state.inverter_data.FR, out.inverter_data.FR);
    copy_inverter_data(shared_state.inverter_data.RL, out.inverter_data.RL);
    copy_inverter_data(shared_state.inverter_data.RR, out.inverter_data.RR);

    //CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor = shared_state.current_sensor_data.twentyfour_volt_sensor;
    out.current_sensor_data.current_sensor_unfiltered = shared_state.current_sensor_data.current_sensor_unfiltered;
    out.current_sensor_data.current_refererence_unfiltered = shared_state.current_sensor_data.current_refererence_unfiltered;

    //DrivetrainDynamicReport_s
    out.drivetrain_data.measuredInverterFLPackVoltage = shared_state.drivetrain_data.measuredInverterFLPackVoltage;
    out.drivetrain_data.measuredSpeeds.FL = shared_state.drivetrain_data.measuredSpeeds.FL;
    out.drivetrain_data.measuredSpeeds.FR = shared_state.drivetrain_data.measuredSpeeds.FR;
    out.drivetrain_data.measuredSpeeds.RL = shared_state.drivetrain_data.measuredSpeeds.RL;
    out.drivetrain_data.measuredSpeeds.RR = shared_state.drivetrain_data.measuredSpeeds.RR;
    out.drivetrain_data.measuredTorques.FL = shared_state.drivetrain_data.measuredTorques.FL;
    out.drivetrain_data.measuredTorques.FR = shared_state.drivetrain_data.measuredTorques.FR;
    out.drivetrain_data.measuredTorques.RL = shared_state.drivetrain_data.measuredTorques.RL;
    out.drivetrain_data.measuredTorques.RR = shared_state.drivetrain_data.measuredTorques.RR;
    out.drivetrain_data.measuredTorqueCurrents.FL = shared_state.drivetrain_data.measuredTorqueCurrents.FL;
    out.drivetrain_data.measuredTorqueCurrents.FR = shared_state.drivetrain_data.measuredTorqueCurrents.FR;
    out.drivetrain_data.measuredTorqueCurrents.RL = shared_state.drivetrain_data.measuredTorqueCurrents.RL;
    out.drivetrain_data.measuredTorqueCurrents.RR = shared_state.drivetrain_data.measuredTorqueCurrents.RR;
    out.drivetrain_data.measuredMagnetizingCurrents.FL = shared_state.drivetrain_data.measuredMagnetizingCurrents.FL;
    out.drivetrain_data.measuredMagnetizingCurrents.FR = shared_state.drivetrain_data.measuredMagnetizingCurrents.FR;
    out.drivetrain_data.measuredMagnetizingCurrents.RL = shared_state.drivetrain_data.measuredMagnetizingCurrents.RL;
    out.drivetrain_data.measuredMagnetizingCurrents.RR = shared_state.drivetrain_data.measuredMagnetizingCurrents.RR;

    //AMSSystemData_s
    out.ams_data.min_cell_voltage = shared_state.ams_data.min_cell_voltage;
    out.ams_data.average_cell_voltage = shared_state.ams_data.average_cell_voltage;
    out.ams_data.max_cell_voltage = shared_state.ams_data.max_cell_voltage;
    out.ams_data.min_temp_celsius = shared_state.ams_data.min_temp_celsius;
    out.ams_data.average_temp_celsius = shared_state.ams_data.average_temp_celsius;
    out.ams_data.max_temp_celsius = shared_state.ams_data.max_temp_celsius;
    out.ams_data.total_pack_voltage = shared_state.ams_data.total_pack_voltage;
    out.ams_data.ams_ok = out.ams_data.ams_ok;

    // Buzzer
    out.buzzer_is_active = shared_state.buzzer_is_active;

    return out;

}

void VCREthernetInterface::receive_pb_msg_acu_all_data(const hytech_msgs_ACUAllData_s &msg_in, VCRData_s &shared_state)
{
    for (uint32_t i = 0; i < msg_in.voltages_count; ++i)
    {
        shared_state.acu_all_data.voltages[i] = msg_in.voltages[i];
    }

    for (uint32_t i = 0; i < msg_in.cell_temperatures_count; ++i)
    {
        shared_state.acu_all_data.cell_temperatures[i] = msg_in.cell_temperatures[i];
    }

    for (uint32_t i = 0; i < msg_in.board_humidities_count; ++i)
    {
        shared_state.acu_all_data.board_humidities[i] = msg_in.board_humidities[i];
    }
}

void VCREthernetInterface::receive_pb_msg_acu_core_data(const hytech_msgs_ACUCoreData_s &msg_in, VCRData_s &shared_state)
{
    shared_state.acu_core_data.avg_cell_voltage = msg_in.avg_cell_voltage;
    shared_state.acu_core_data.max_cell_temp = msg_in.max_cell_temp;
    shared_state.acu_core_data.min_cell_voltage = msg_in.min_cell_voltage;
    shared_state.acu_core_data.pack_voltage = msg_in.pack_voltage;
}

void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in, VCRData_s &shared_state)
{
    //TODO: Finish this function. This function could parse the message and put it into shared_state, but depending
    //      on where things are defined, it might be cleaner for this function to simply return the new data. I do
    //      not know yet. Definitely worth asking Ben.
}

void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFData_s &msg_in, VCRData_s &shared_state)
{
    //PedalsSystemData_s
    shared_state.pedals_system_data.accel_is_implausible = msg_in.pedals_system_data.accel_is_implausible;
    shared_state.pedals_system_data.brake_is_implausible = msg_in.pedals_system_data.brake_is_implausible;
    shared_state.pedals_system_data.brake_is_pressed = msg_in.pedals_system_data.brake_is_pressed;
    shared_state.pedals_system_data.accel_is_pressed = msg_in.pedals_system_data.accel_is_pressed;
    shared_state.pedals_system_data.mech_brake_is_active = msg_in.pedals_system_data.mech_brake_is_active;
    shared_state.pedals_system_data.brake_and_accel_pressed_implausibility_high = msg_in.pedals_system_data.brake_and_accel_pressed_implausibility_high;
    shared_state.pedals_system_data.implausibility_has_exceeded_max_duration = msg_in.pedals_system_data.implausibility_has_exceeded_max_duration;
    shared_state.pedals_system_data.accel_percent = msg_in.pedals_system_data.accel_percent;
    shared_state.pedals_system_data.brake_percent = msg_in.pedals_system_data.brake_percent;
    shared_state.pedals_system_data.regen_percent = msg_in.pedals_system_data.regen_percent;

    //DashInputState_s
    shared_state.dash_input_state.data_btn_is_pressed = msg_in.dash_input_state.data_btn_is_pressed;
    shared_state.dash_input_state.dial_state = (ControllerMode_e) msg_in.dash_input_state.dial_state;
    shared_state.dash_input_state.dim_btn_is_pressed = msg_in.dash_input_state.dim_btn_is_pressed;
    shared_state.dash_input_state.left_paddle_is_pressed = msg_in.dash_input_state.left_paddle_is_pressed;
    shared_state.dash_input_state.right_paddle_is_pressed = msg_in.dash_input_state.right_paddle_is_pressed;
    shared_state.dash_input_state.mc_reset_btn_is_pressed = msg_in.dash_input_state.mc_reset_btn_is_pressed;
    shared_state.dash_input_state.mode_btn_is_pressed = msg_in.dash_input_state.mode_btn_is_pressed;
    shared_state.dash_input_state.preset_btn_is_pressed = msg_in.dash_input_state.preset_btn_is_pressed;
    shared_state.dash_input_state.start_btn_is_pressed = msg_in.dash_input_state.start_btn_is_pressed;
}
	
void VCREthernetInterface::copy_inverter_data(InverterData_s &original, hytech_msgs_InverterData_s &destination)
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