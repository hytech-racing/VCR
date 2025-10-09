/******************************************************************************
 * @file    InverterInterface.h
 * @brief   Header for any receive/send to the inverters
 ******************************************************************************/

 /******************************************************************************
 * Includes
 ******************************************************************************/
#include <InverterInterface.h>
#include "VCRCANInterfaceImpl.h"
#include <Arduino.h>

/******************************************************************************
 * Public Method Definitions
 ******************************************************************************/
/**
 * Getters for status data
 * @return the requested status struct
 */
inverter_status_s InverterInterface::getStatus() {
    inverter_status_s inverter_status = _feedback_data.status;
    _feedback_data.status.new_data = false;
    return inverter_status;
}

/**
 * Getters for temp data
 * @return the requested temp struct
 */
inverter_temps_s InverterInterface::getTemps() {
    inverter_temps_s inverter_temps = _feedback_data.temps;
    _feedback_data.temps.new_data = false;
    return inverter_temps;
}

/**
 * Getters for power data
 * @return the requested power struct
 */
inverter_power_s InverterInterface::getPower() {
    inverter_power_s inverter_power = _feedback_data.power;
    _feedback_data.power.new_data = false;
    return inverter_power;
}

/**
 * Getters for motor mechanics data
 * @return the requested motor mechanics struct
 */
inverter_motor_mechanics_s InverterInterface::getMotorMechanics() {
    inverter_motor_mechanics_s inverter_motor_mechanics = _feedback_data.motor_mechanics;
    _feedback_data.motor_mechanics.new_data = false;
    return inverter_motor_mechanics;
}

/**
 * Getters for the control params
 * @return the requested control params struct
 */
inverter_control_feedback_s InverterInterface::getControlFeedback() {
    inverter_control_feedback_s inverter_control_feedback = _feedback_data.control_feedback;
    _feedback_data.control_feedback.new_data = false;
    return inverter_control_feedback;
}

/**
 * Receives an inverter status message
 * @param can_msg the CAN message to process
 * @param curr_millis the current time in milliseconds
 */
void InverterInterface::receiveInverterStatus(const CAN_message_t &can_msg, unsigned long curr_millis)
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
    _feedback_data.status.hv_present = _feedback_data.status.dc_bus_voltage > _inverter_params.high_voltage_threshold;

    _feedback_data.status.new_data = true;
    _feedback_data.status.last_recv_millis = curr_millis;
}

/**
 * Receives an inverter temps message
 * @param can_msg the CAN message to process
 * @param curr_millis the current time in milliseconds
 */
void InverterInterface::receiveInverterTemps(const CAN_message_t &can_msg, unsigned long curr_millis)
{

    // Unpack the message
    INV1_TEMPS_t unpacked_msg;
    Unpack_INV1_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.temps.igbt_temp_celcius = HYTECH_igbt_temp_ro_fromS(unpacked_msg.igbt_temp_ro);
    _feedback_data.temps.inverter_temp_celcius = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
    _feedback_data.temps.motor_temp_celcius = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);

    _feedback_data.temps.new_data = true;
    _feedback_data.temps.last_recv_millis = curr_millis;
}


/**
 * Receives an inverter dynamics message
 * @param can_msg the CAN message to process
 * @param curr_millis the current time in milliseconds
 */
void InverterInterface::recieveInverterDynamics(const CAN_message_t &can_msg, unsigned long curr_millis) 
{
    // Unpack the message
    INV1_DYNAMICS_t unpacked_msg;
    Unpack_INV1_DYNAMICS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.motor_mechanics.actual_power_watts = unpacked_msg.actual_power_w; // NOLINT (watts)
    _feedback_data.motor_mechanics.actual_torque_nm = HYTECH_actual_torque_nm_ro_fromS(unpacked_msg.actual_torque_nm_ro);
    _feedback_data.motor_mechanics.actual_speed_rpm = unpacked_msg.actual_speed_rpm;
    _feedback_data.motor_mechanics.new_data = true;
    _feedback_data.motor_mechanics.last_recv_millis = curr_millis;
}


/**
 * Receives an inverter power message
 * @param can_msg the CAN message to process
 * @param curr_millis the current time in milliseconds
 */
void InverterInterface::receiveInverterPower(const CAN_message_t &can_msg, unsigned long curr_millis) 
{
    // Unpack the message
    INV1_POWER_t unpacked_msg;
    Unpack_INV1_POWER_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.power.active_power_watts = unpacked_msg.active_power_w; // NOLINT (watts)
    _feedback_data.power.reactive_power_watts = unpacked_msg.reactive_power_var; // NOLINT (watts)

    _feedback_data.power.new_data = true;
    _feedback_data.power.last_recv_millis = curr_millis;
}


/**
 * Receives an inverter feedbck message
 * @param can_msg the CAN message to process
 * @param curr_millis the current time in milliseconds
 */
void InverterInterface::receiveInverterFeedback(const CAN_message_t &can_msg, unsigned long curr_millis) 
{
    // Unpack the message
    INV1_FEEDBACK_t unpacked_msg;
    Unpack_INV1_FEEDBACK_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _feedback_data.control_feedback.speed_control_kp = unpacked_msg.speed_control_kp;
    _feedback_data.control_feedback.speed_control_ki = unpacked_msg.speed_control_ki;
    _feedback_data.control_feedback.speed_control_kd = unpacked_msg.speed_control_kd;

    _feedback_data.control_feedback.new_data = true;
    _feedback_data.control_feedback.last_recv_millis = curr_millis;
}


/**
 * Sends an inverter Setpoint message over CAN. Doesn't need to
 * take in any parameters because the structs are stored internally
 * to the interface
 */
void InverterInterface::sendInverterSetpointCommand() 
{
    INV1_CONTROL_INPUT_t msg_out;

    msg_out.speed_setpoint_rpm = _inverter_control_inputs.speed_rpm_setpoint;
    msg_out.positive_torque_limit_ro = HYTECH_positive_torque_limit_ro_toS(_inverter_control_inputs.positive_torque_limit);
    msg_out.negative_torque_limit_ro = HYTECH_negative_torque_limit_ro_toS(_inverter_control_inputs.negative_torque_limit);

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_INPUT_hytech, VCRCANInterfaceImpl::inverter_can_tx_buffer, _inverter_ids.inverter_control_input_id);
}

/**
 * Sends an inverter cotrol word over CAN. Doesn't need to
 * take in any parameters because the structs are stored internally
 * to the interface
 */
void InverterInterface::sendInverterControlWord() 
{
    INV1_CONTROL_WORD_t msg_out;

    msg_out.driver_enable = _inverter_control_word.driver_enable;
    msg_out.hv_enable = _inverter_control_word.hv_enable;
    msg_out.inverter_enable = _inverter_control_word.inverter_enable;
    msg_out.remove_error = _inverter_control_word.remove_error;

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_WORD_hytech, VCRCANInterfaceImpl::inverter_can_tx_buffer, _inverter_ids.inverter_control_word_id);
    
}

/**
 * Sends inverter control params over CAN. Doesn't need to
 * take in any parameters because the structs are stored internally
 * to the interface
 */
void InverterInterface::sendInverterControlParams() 
{
    INV1_CONTROL_PARAMETER_t msg_out;

    msg_out.speed_control_kd = _inverter_control_params.speed_control_kp;
    msg_out.speed_control_ki = _inverter_control_params.speed_control_ki;
    msg_out.speed_control_kd = _inverter_control_params.speed_control_kd;

    CAN_util::enqueue_msg(&msg_out, &Pack_INV1_CONTROL_PARAMETER_hytech, VCRCANInterfaceImpl::inverter_can_tx_buffer, _inverter_ids.inverter_control_parameter_id);
}



/**
 * Modifies the desired speed and torque limit for the inverter, populating the struct that is sent
 * by the sendInverterSetpointCommand method
 * @param desired_rpm the desired speed in RPM
 * @param torque_limit_nm the desired torque limit in Nm
 */
void InverterInterface::setSpeed(float desired_rpm, float torque_limit_nm) 
{
    _inverter_control_inputs.speed_rpm_setpoint = static_cast<int16_t>(desired_rpm);
    _inverter_control_inputs.positive_torque_limit = ::fabs(torque_limit_nm);
    _inverter_control_inputs.negative_torque_limit = -1.0f * ::fabs(torque_limit_nm);
}

/**
 * Sets the inverter to idle (0 RPM, 0 Nm)
 */
void InverterInterface::setIdle() 
{
    _inverter_control_inputs.negative_torque_limit = 0; 
    _inverter_control_inputs.positive_torque_limit = 0;
    _inverter_control_inputs.speed_rpm_setpoint = 0;
}

/**
 * Sets the inverter control word
 * @param control_word the desired control word
 */
void InverterInterface::setInverterControlWord(inverter_control_word_s control_word) 
{
    _inverter_control_word.driver_enable = control_word.driver_enable;
    _inverter_control_word.hv_enable = control_word.hv_enable;
    _inverter_control_word.inverter_enable = control_word.inverter_enable;
    _inverter_control_word.remove_error = control_word.remove_error;
}