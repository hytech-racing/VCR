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


InverterControlFeedback_s InverterInterface::get_control_params() {
    _feedback_data.control_feedback.new_data = false;
    return _feedback_data.control_feedback;
}


/**
 * receiving CAN messages
 */

void InverterInterface::receive_INV_STATUS(CAN_message_t &can_msg)
{
    // Unpack the message
    INV1_STATUS_t unpacked_msg;
    Unpack_INV1_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    
    // Update inverter interface with new data
    _feedback_data.status.connected = true; // Will set to true once first CAN message has been received
    _feedback_data.status.system_ready = unpacked_msg.system_ready;
    _feedback_data.status.error = unpacked_msg.error;
    _feedback_data.status.warning = unpacked_msg.warning;
    _feedback_data.status.quit_dc_on = unpacked_msg.quit_dc_on;
    _feedback_data.status.dc_on = unpacked_msg.dc_on;
    _feedback_data.status.quit_inverter_on = unpacked_msg.quit_inverter_on;
    _feedback_data.status.derating_on = unpacked_msg.derating_on;
    _feedback_data.status.dc_bus_voltage = unpacked_msg.dc_bus_voltage;
    _feedback_data.status.diagnostic_number = unpacked_msg.diagnostic_number;
    _feedback_data.status.hv_present = _feedback_data.status.dc_bus_voltage > _inverter_params.MINIMUM_HV_VOLTAGE;
}

void InverterInterface::receive_INV_TEMPS(CAN_message_t &can_msg)
{

    // Unpack the message
    INV1_TEMPS_t unpacked_msg;
    Unpack_INV1_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.temps.igbt_temp = HYTECH_igbt_temp_ro_fromS(unpacked_msg.igbt_temp_ro);
    _feedback_data.temps.inverter_temp = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
    _feedback_data.temps.motor_temp = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);
    _feedback_data.temps.new_data = true;

}

void InverterInterface::receive_INV_DYNAMICS(CAN_message_t &can_msg) 
{

    // Unpack the message
    INV1_DYNAMICS_t unpacked_msg;
    Unpack_INV1_DYNAMICS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.motor_mechanics.actual_power = unpacked_msg.actual_power_w;
    _feedback_data.motor_mechanics.actual_torque = unpacked_msg.actual_torque_nm;
    _feedback_data.motor_mechanics.actual_speed = unpacked_msg.actual_speed_rpm;
    _feedback_data.motor_mechanics.new_data = true;

}

void InverterInterface::receive_INV_POWER(CAN_message_t &can_msg) 
{
    // Unpack the message
    INV1_POWER_t unpacked_msg;
    Unpack_INV1_POWER_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.power.active_power = unpacked_msg.active_power_w;
    _feedback_data.power.reactive_power = unpacked_msg.reactive_power_var;
    _feedback_data.power.new_data = true;

}

void InverterInterface::receive_INV_FEEDBACK(CAN_message_t &can_msg) 
{
    // Unpack the message
    INV1_FEEDBACK_t unpacked_msg;
    Unpack_INV1_FEEDBACK_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.control_feedback.speed_control_kp = unpacked_msg.speed_control_kp;
    _feedback_data.control_feedback.speed_control_ki = unpacked_msg.speed_control_ki;
    _feedback_data.control_feedback.speed_control_kd = unpacked_msg.speed_control_kd;
    _feedback_data.control_feedback.new_data = true;

}

/**
 * Sending CAN messages
 */

void InverterInterface::send_INV_SETPOINT_COMMAND() 
{
    INV1_CONTROL_INPUT_t msg_out;

    msg_out.speed_setpoint_rpm = _inverter_control_inputs.speed_rpm_setpoint;
    msg_out.positive_torque_limit_ro = _inverter_control_inputs.positive_torque_limit;
    msg_out.negative_torque_limit_ro = _inverter_control_inputs.negative_torque_limit;

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_INPUT_hytech, VCRCANInterfaceImpl::telem_can_tx_buffer, inverter_ids.inv_control_input_id);
}

void InverterInterface::send_INV_CONTROL_WORD() 
{
    INV1_CONTROL_WORD_t msg_out;

    msg_out.driver_enable = _inverter_control_word.driver_enable;
    msg_out.hv_enable = _inverter_control_word.hv_enable;
    msg_out.inverter_enable = _inverter_control_word.inverter_enable;
    msg_out.remove_error = _inverter_control_word.remove_error;

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_WORD_hytech, VCRCANInterfaceImpl::telem_can_tx_buffer, inverter_ids.inv_control_word_id);
}

void InverterInterface::send_INV_CONTROL_PARAMS() 
{
    INV1_CONTROL_PARAMETER_t msg_out;

    msg_out.speed_control_kd = _inverter_control_params.speed_control_kp;
    msg_out.speed_control_ki = _inverter_control_params.speed_control_ki;
    msg_out.speed_control_kd = _inverter_control_params.speed_control_kd;

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_PARAMETER_hytech, VCRCANInterfaceImpl::telem_can_tx_buffer, inverter_ids.inv_control_parameter_id);
}

/**
 * Methods for use as inverter functs
 */

void InverterInterface::set_speed(float desired_rpm, float torque_limit_nm) 
{
    _inverter_control_inputs.speed_rpm_setpoint = desired_rpm;

    float converted_torque = std::abs(torque_limit_nm * (1000/9.8));

    _inverter_control_inputs.positive_torque_limit = converted_torque;
    _inverter_control_inputs.negative_torque_limit = -converted_torque;
}

void InverterInterface::set_idle() 
{
    _inverter_control_inputs.negative_torque_limit = 0; 
    _inverter_control_inputs.positive_torque_limit = 0;
    _inverter_control_inputs.speed_rpm_setpoint = 0;
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
