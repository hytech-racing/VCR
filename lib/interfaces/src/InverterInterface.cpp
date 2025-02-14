#include <InverterInterface.h>

/**
 * Getters for the data
 */

InverterStatus_s InverterInterface::get_status() {
    _feedback_data.status.new_data = false;
    return _feedback_data.status;
}

InverterTemps_s InverterInterface::get_temps() {
    _feedback_data.temps.new_data = false;
    return _feedback_data.temps;
}

InverterPower_s InverterInterface::get_power() {
    _feedback_data.power.new_data = false;
    return _feedback_data.power;
}

MotorMechanics_s InverterInterface::get_motor_mechanics() {
    _feedback_data.motor_mechanics.new_data = false;
    return _feedback_data.motor_mechanics;
}


InverterControlParams_s InverterInterface::get_control_params() {
    _feedback_data.control_params.new_data = false;
    return _feedback_data.control_params;
}


/**
 * Recieving CAN messages
 */

void InverterInterface::recieve_MCI_STATUS(CAN_message_t &can_msg)
{
    // Unpack the message
    MCI1_STATUS_t unpacked_msg;
    Unpack_MCI1_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    
    // Update inverter interface with new data
    _feedback_data.status.connected = true; // Will set to true once first CAN message has been recieved
    _feedback_data.status.system_ready = unpacked_msg.system_ready;
    _feedback_data.status.error = unpacked_msg.error;
    _feedback_data.status.warning = unpacked_msg.warning;
    _feedback_data. status.quit_dc_on = unpacked_msg.quit_dc_on;
    _feedback_data.status.dc_on = unpacked_msg.dc_on;
    _feedback_data.status.quit_inverter_on = unpacked_msg.quit_inverter_on;
    _feedback_data.status.derating_on = unpacked_msg.derating_on;
    _feedback_data.status.dc_bus_voltage = unpacked_msg.dc_bus_voltage;
    _feedback_data.status.diagnostic_number = unpacked_msg.diagnostic_number;
    _feedback_data.status.hv_present = _feedback_data.status.dc_bus_voltage > _inverter_params.MINIMUM_HV_VOLTAGE;
}

void InverterInterface::recieve_MCI_TEMPS(CAN_message_t &can_msg)
{

    Serial.println("Recieved temps data!");

    // Unpack the message
    MCI1_TEMPS_t unpacked_msg;
    Unpack_MCI1_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.temps.igbt_temp = HYTECH_igbt_temp_ro_fromS(unpacked_msg.igbt_temp_ro);
    _feedback_data.temps.inverter_temp = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
    _feedback_data.temps.motor_temp = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _feedback_data.temps.new_data = true;

}

void InverterInterface::recieve_MCI_DYNAMICS(CAN_message_t &can_msg) 
{

    // Unpack the message
    MCI1_DYNAMICS_t unpacked_msg;
    Unpack_MCI1_DYNAMICS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.motor_mechanics.actual_power = unpacked_msg.actual_power_w;
    _feedback_data.motor_mechanics.actual_torque = unpacked_msg.actual_torque_nm;
    _feedback_data.motor_mechanics.actual_speed = unpacked_msg.actual_speed_rpm;
    _feedback_data.motor_mechanics.new_data = true;

}

void InverterInterface::recieve_MCI_POWER(CAN_message_t &can_msg) 
{
    // Unpack the message
    MCI1_POWER_t unpacked_msg;
    Unpack_MCI1_POWER_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.power.active_power = unpacked_msg.active_power_w;
    _feedback_data.power.reactive_power = unpacked_msg.reactive_power_var;
    _feedback_data.power.new_data = true;

}

void InverterInterface::recieve_MCI_FEEDBACK(CAN_message_t &can_msg) 
{
    // Unpack the message
    MCI1_FEEDBACK_t unpacked_msg;
    Unpack_MCI1_FEEDBACK_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.control_params.speed_control_kp = unpacked_msg.speed_control_kp;
    _feedback_data.control_params.speed_control_ki = unpacked_msg.speed_control_ki;
    _feedback_data.control_params.speed_control_kd = unpacked_msg.speed_control_kd;
    _feedback_data.control_params.new_data = true;

}

/**
 * Sending CAN messages
 */

template <typename U>
void InverterInterface::enqueue_new_CAN(U *structure, uint32_t (*pack_function)(U *, uint8_t *, uint8_t *, uint8_t *), uint32_t id)
{
    CAN_message_t can_msg;
    pack_function(structure, can_msg.buf, &can_msg.len, (uint8_t *)&can_msg.flags.extended);
    can_msg.id = id;
    uint8_t buf[sizeof(CAN_message_t)] = {};
    memmove(buf, &can_msg, sizeof(CAN_message_t));
    msg_queue_->push_back(buf, sizeof(CAN_message_t));
}

void InverterInterface::send_MC_SETPOINT_COMMAND() 
{
    MC1_SETPOINTS_COMMAND_t msg_out;

    msg_out.speed_setpoint_rpm = _inverter_setpoints.speed_rpm_setpoint;
    msg_out.positive_torque_limit_ro = _inverter_setpoints.positive_torque_limit;
    msg_out.negative_torque_limit_ro = _inverter_setpoints.negative_torque_limit;

    enqueue_new_CAN<MC1_SETPOINTS_COMMAND_t>(&msg_out, &Pack_MC1_SETPOINTS_COMMAND_hytech, inverter_ids.mc_setpoint_commands_id);
}

void InverterInterface::send_MC_CONTROL_WORD() 
{
    // TODO edit PCAN project to finish this
}

/**
 * Methods for use as inverter functs
 */

void InverterInterface::set_speed(float desired_rpm, float torque_limit_nm) 
{
    _inverter_setpoints.speed_rpm_setpoint = desired_rpm;
    _inverter_setpoints.positive_torque_limit = torque_limit_nm;
    _inverter_setpoints.negative_torque_limit = -torque_limit_nm;
}

void InverterInterface::set_torque(float torque_nm) 
{
    // TODO set desired rpm to max? 
    _inverter_setpoints.positive_torque_limit = torque_nm;
    _inverter_setpoints.negative_torque_limit = -torque_nm;
}

void InverterInterface::set_idle() 
{
    _inverter_setpoints.negative_torque_limit = 0; 
    _inverter_setpoints.positive_torque_limit = 0;
    _inverter_setpoints.speed_rpm_setpoint = 0;
}

void InverterInterface::set_inverter_control_word(InverterControlWord_s control_word) 
{
    _inverter_control_word.driver_enable = control_word.driver_enable;
    _inverter_control_word.hv_enable = control_word.hv_enable;
    _inverter_control_word.inverter_enable = control_word.inverter_enable;
    _inverter_control_word.remove_error = control_word.remove_error;
}

InverterStatus_s InverterInterface::get_inverter_status() 
{
    return _feedback_data.status;
}