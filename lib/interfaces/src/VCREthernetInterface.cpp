#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include <Arduino.h>

hytech_msgs_VCRSystemData_s VCREthernetInterface::make_vcrsystemdata_msg(const VCRSystemData_s &shared_state)
{
	hytech_msgs_VCRSystemData_s out;

    //RearLoadCellsFiltered_s
    out.rear_loadcells_filtered.RL_loadcell_filtered_pounds;
    out.rear_loadcells_filtered.RR_loadcell_filtered_pounds;
    out.rear_loadcells_filtered.rear_loadcell_FIR_is_saturated;

    out.rear_loadcells_filtered = {shared_state.rear_loadcells_filtered.RL_loadcell_filtered_pounds,
                                   shared_state.rear_loadcells_filtered.RR_loadcell_filtered_pounds,
                                   shared_state.rear_loadcells_filtered.rear_loadcell_FIR_is_saturated};

    
    //FrontLoadCellsFiltered_s
    out.front_loadcells_filtered.FL_loadcell_filtered_pounds;
    out.front_loadcells_filtered.FR_loadcell_filtered_pounds;
    out.front_loadcells_filtered.front_loadcell_FIR_is_saturated;

    out.front_loadcells_filtered = {shared_state.front_loadcells_filtered.FL_loadcell_filtered_pounds,
                                    shared_state.front_loadcells_filtered.FR_loadcell_filtered_pounds,
                                    shared_state.front_loadcells_filtered.front_loadcell_FIR_is_saturated};

    

    //FrontSusPotsFiltered_s
    out.front_suspots_filtered.FL_sus_pot_filtered_analog;
    out.front_suspots_filtered.FR_sus_pot_filtered_analog;
    out.front_suspots_filtered.front_loadcell_FIR_is_saturated;

    out.front_suspots_filtered = {shared_state.front_suspots_filtered.FL_sus_pot_filtered_analog,
                                  shared_state.front_suspots_filtered.FR_sus_pot_filtered_analog,
                                  shared_state.front_suspots_filtered.front_loadcell_FIR_is_saturated};


    //SteeringFiltered_s
    out.steering_filtered.steering_filtered_degrees;
    out.steering_filtered.steering_FIR_is_saturated;

    out.steering_filtered = {shared_state.steering_filtered.steering_filtered_degrees,
                             shared_state.steering_filtered.steering_FIR_is_saturated};
 
    
    //DashDisplayState_s
    out.dash_display.dash_data;
    out.dash_display = {shared_state.dash_display.dash_data};

    return out;

}

hytech_msgs_VCRInterfaceData_s VCREthernetInterface::make_vcrinterfacedata_msg(const VCRInterfaceData_s &shared_state)
{
    hytech_msgs_VCRInterfaceData_s out;

    //RearLoadCellsUnfiltered_s
    out.rear_loadcells_unfiltered.RL_loadcell_unfiltered_pounds;
    out.rear_loadcells_unfiltered.RR_loadcell_unfiltered_pounds;

    out.rear_loadcells_unfiltered = {shared_state.rear_loadcells_unfiltered.RL_loadcell_unfiltered_pounds,
                                   shared_state.rear_loadcells_unfiltered.RR_loadcell_unfiltered_pounds};

    
    //RearSusPotsUnfiltered_s
    out.rear_suspots_unfiltered.RL_sus_pot_unfiltered_analog;
    out.rear_suspots_unfiltered.RR_sus_pot_unfiltered_analog;
    //out.front_loadcells_filtered.front_loadcell_FIR_is_saturated;

    out.front_loadcells_filtered = {shared_state.rear_suspots_unfiltered.RL_sus_pot_unfiltered_analog,
                                    shared_state.rear_suspots_unfiltered.RR_sus_pot_unfiltered_analog};

    

    //VectorNavData_s
    out.vectornav_data.velocity_x;
    out.vectornav_data.velocity_y;
    out.vectornav_data.velocity_z;
    out.vectornav_data.linear_accel_x;
    out.vectornav_data.linear_accel_y;
    out.vectornav_data.linear_accel_z;
    out.vectornav_data.uncompLinear_accel[3]; //idk if this format works
    out.vectornav_data.yaw;
    out.vectornav_data.pitch;
    out.vectornav_data.roll;
    out.vectornav_data.latitude;
    out.vectornav_data.longitude;
    out.vectornav_data.ecef_coords[3]; //same with this
    out.vectornav_data.gps_time;
    out.vectornav_data.vn_status;
    out.vectornav_data.angular_rates;

    out.vectornav_data = {shared_state.vectornav_data.velocity_x,
                          shared_state.vectornav_data.velocity_y,
                          shared_state.vectornav_data.velocity_z,
                          shared_state.vectornav_data.linear_accel_x,
                          shared_state.vectornav_data.linear_accel_y,
                          shared_state.vectornav_data.linear_accel_z,
                          shared_state.vectornav_data.uncompLinear_accel[3], //idk if this format works
                          shared_state.vectornav_data.yaw,
                          shared_state.vectornav_data.pitch,
                          shared_state.vectornav_data.roll,
                          shared_state.vectornav_data.latitude,
                          shared_state.vectornav_data.longitude,
                          shared_state.vectornav_data.ecef_coords[3], //same with this
                          shared_state.vectornav_data.gps_time,
                          shared_state.vectornav_data.vn_status,
                          shared_state.vectornav_data.angular_rates};
    


    //CurrentSensorData_s
    out.current_sensor_data.twentyfour_volt_sensor;
    out.current_sensor_data.current_sensor_unfiltered;
    out.current_sensor_data.current_refererence_unfiltered;

    out.current_sensor_data = {shared_state.current_sensor_data.twentyfour_volt_sensor,
                               shared_state.current_sensor_data.current_sensor_unfiltered,
                               shared_state.current_sensor_data.current_refererence_unfiltered};

    
    //ShutdownSensingData_s
    out.shutdown_sensing_data.i_shutdown_in;
    out.shutdown_sensing_data.j_bspd_relay;
    out.shutdown_sensing_data.k_watchdog_relay;
    out.shutdown_sensing_data.l_bms_relay;
    out.shutdown_sensing_data.m_imd_relay;

    out.shutdown_sensing_data.bspd_is_ok;
    out.shutdown_sensing_data.watchdog_is_ok;
    out.shutdown_sensing_data.bms_is_ok;
    out.shutdown_sensing_data.imd_is_ok;


    out.dash_display = {shared_state.shutdown_sensing_data.i_shutdown_in,
                        shared_state.shutdown_sensing_data.j_bspd_relay,
                        shared_state.shutdown_sensing_data.k_watchdog_relay,
                        shared_state.shutdown_sensing_data.l_bms_relay,
                        shared_state.shutdown_sensing_data.m_imd_relay,

                        shared_state.shutdown_sensing_data.bspd_is_ok,
                        shared_state.shutdown_sensing_data.watchdog_is_ok,
                        shared_state.shutdown_sensing_data.bms_is_ok,
                        shared_state.shutdown_sensing_data.imd_is_ok};

    //EthernetLinkData_s
    out.ethernet_is_linked.acu_link;
    out.ethernet_is_linked.drivebrain_link;
    out.ethernet_is_linked.vcf_link;
    out.ethernet_is_linked.teensy_link;
    out.ethernet_is_linked.debug_link;
    out.ethernet_is_linked.ubiquiti_link;

    out.ethernet_is_linked = {shared_state.ethernet_is_linked.acu_link,
                              shared_state.ethernet_is_linked.drivebrain_link,
                              shared_state.ethernet_is_linked.vcf_link,
                              shared_state.ethernet_is_linked.teensy_link,
                              shared_state.ethernet_is_linked.debug_link,
                              shared_state.ethernet_is_linked.ubiquiti_link};


    //veh_vec_inverter
    
    out.inverter_data.system_ready;
    out.inverter_data.error;
    out.inverter_data.warning;
    out.inverter_data.quit_dc_on;
    out.inverter_data.dc_on;
    out.inverter_data.quit_inverter_on;
    out.inverter_data.inverter_on;
    out.inverter_data.derating_on;
    out.inverter_data.speed_rpm;
    out.inverter_data.actual_motor_torque;
    out.inverter_data.commanded_torque;
    out.inverter_data.motor_temp;
    out.inverter_data.inverter_temp;
    out.inverter_data.diagnostic_number;
    out.inverter_data.igbt_temp;
    out.inverter_data.dc_bus_voltage;
    out.inverter_data.actual_power;
    out.inverter_data.feedback_torque;

    out.inverter_data = {shared_state.inverter_data.system_ready,
                         shared_state.inverter_data.error,
                         shared_state.inverter_data.warning,
                         shared_state.inverter_data.quit_dc_on,
                         shared_state.inverter_data.dc_on,
                         shared_state.inverter_data.quit_inverter_on,
                         shared_state.inverter_data.inverter_on,
                         shared_state.inverter_data.derating_on,
                         shared_state.inverter_data.speed_rpm,
                         shared_state.inverter_data.actual_motor_torque,
                         shared_state.inverter_data.commanded_torque,
                         shared_state.inverter_data.motor_temp,
                         shared_state.inverter_data.inverter_temp,
                         shared_state.inverter_data.diagnostic_number,
                         shared_state.inverter_data.igbt_temp,
                         shared_state.inverter_data.dc_bus_voltage,
                         shared_state.inverter_data.actual_power,
                         shared_state.inverter_data.feedback_torque};

    return out;


}

//
void VCREthernetInterface::receive_pb_msg_acu(const hytech_msgs_BMSData &msg_in)
{
    std::array<std::array<std::optional<volt>, 12>, num_chips> voltages;
    std::array<celcius, 4 * num_chips> cell_temperatures;
    std::array<float, num_humidity_sensors> humidity;
    std::array<float, num_board_thermistors> board_temperatures;

    //TEMPORARY!!! ____________________________________________________
    _latest_acu_data.voltages = msg_in.voltages;
    _latest_acu_data.cell_temperatures = msg_in.cell_temperatures
    _latest_acu_data.humidity = msg_in.humidity;
    _latest_acu_data.board_temperatures = msg_in.board_temperatures;
    //_________________________________________________________________



    _latest_acu_data.min_voltage = msg_in.min_voltage;
    _latest_acu_data.max_voltage = msg_in.max_voltage;
    _latest_acu_data.min_voltage_cell_id = msg_in.min_voltage_cell_id;
    _latest_acu_data.max_voltage_cell_id = msg_in.max_voltage_cell_id;
    _latest_acu_data.max_board_temperature_segment_id = msg_in.max_board_temperature_segment_id;
    _latest_acu_data.max_humidity_segment_id = msg_in.max_humidity_segment_id;
    _latest_acu_data.max_cell_temperature_cell_id = msg_in.max_cell_temperature_cell_id;
    _latest_acu_data.total_voltage = msg_in.total_voltage;
    _latest_acu_data.average_cell_temperature = msg_in.average_cell_temperature;
    

}

void VCREthernetInterface::receive_pb_msg_db(const hytech_msgs_MCUCommandData &msg_in)
{
    veh_vec<float> speed_desired_rpms(msg_in.desired_rpms.FL, msg_in.desired_rpms.FR, msg_in.desired_rpms.RL, msg_in.desired_rpms.RR);
    veh_vec<float> torque_lim_nm(msg_in.torque_limit_nm.FL, msg_in.torque_limit_nm.FR, msg_in.torque_limit_nm.RL, msg_in.torque_limit_nm.RR);

    _latest_db_data.desired_rpms = speed_desired_rpms; 
    _latest_db_data.torque_limit_nm = torque_lim_nm; 

    _latest_db_data.prev_MCU_recv_millis = msg_in.prev_MCU_recv_millis; 
}

void VCREthernetInterface::receive_pb_msg_vcf(const hytech_msgs_VCFSystemData_s &msg_in)
{
     //deep
     //PedalsSystemData_s
    _latest_vcf_data.pedals_system_data.accel_is_implausible = msg_in.pedals_system_data.accel_is_implausible; // --> through regen_percent
    _latest_vcf_data.pedals_system_data.brake_is_implausible = msg_in.pedals_system_data.brake_is_implausible;
    _latest_vcf_data.pedals_system_data.brake_is_pressed = msg_in.pedals_system_data.brake_is_pressed;
    _latest_vcf_data.pedals_system_data.accel_is_pressed = msg_in.pedals_system_data.accel_is_pressed
    _latest_vcf_data.pedals_system_data.mech_brake_is_active = msg_in.pedals_system_data.mech_brake_is_active;
    _latest_vcf_data.pedals_system_data.brake_and_accel_pressed_implausibility_high = msg_in.pedals_system_data.brake_and_accel_pressed_implausibility_high
    _latest_vcf_data.pedals_system_data.implausibility_has_exceeded_max_duration = msg_in.pedals_system_data.implausibility_has_exceeded_max_duration;
    _latest_vcf_data.pedals_system_data.accel_percent = msg_in.pedals_system_data.accel_percent;
    _latest_vcf_data.pedals_system_data.brake_percent = msg_in.pedals_system_data.brake_percent;
    _latest_vcf_data.pedals_system_data.regen_percent = msg_in.pedals_system_data.regen_percent;

    //struct FrontLoadCellsFiltered_s

   _latest_vcf_data.front_loadcells_filtered.FL_loadcell_filtered_pounds = msg_in.front_loadcells_filtered.FL_loadcell_filtered_pounds;
   _latest_vcf_data.front_loadcells_filtered.FR_loadcell_filtered_pounds = msg_in.front_loadcells_filtered.FR_loadcell_filtered_pounds;
   _latest_vcf_data.front_loadcells_filtered.front_loadcell_FIR_is_saturated = msg_in.front_loadcells_filtered.front_loadcell_FIR_is_saturated;

   //FrontSusPotsFiltered_s
   _latest_vcf_data.front_suspots_filtered.FL_sus_pot_filtered_analog = msg_in.front_suspots_filtered.FL_sus_pot_filtered_analog;
   _latest_vcf_data.front_suspots_filtered.FR_sus_pot_filtered_analog = msg_in.front_suspots_filtered.FR_sus_pot_filtered_analog;
   _latest_vcf_data.front_suspots_filtered.front_loadcell_FIR_is_saturated = msg_in.front_suspots_filtered.front_loadcell_FIR_is_saturated;

   //SteeringFiltered_s steering_filtered

   _latest_vcf_data.steering_filtered.steering_filtered_degrees = msg_in.steering_filtered.steering_filtered_degrees;
   _latest_vcf_data.steering_filtered.steering_FIR_is_saturated = msg_in.steering_filtered.steering_FIR_is_saturated;

   //DashDisplayState_s dash_display
   _latest_vcf_data.dash_display.dash_data = msg_in.dash_display.dash_data;
}
	



