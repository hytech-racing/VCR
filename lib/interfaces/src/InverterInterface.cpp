#include <InverterInterface.h>

// TODO Parameterize this so it works for all motor controllers

/** 
 * Request change of state
 */

void InverterInterface::set_inverter_setpoints(
    bool inverter_enable,
    bool hv_enable,
    bool driver_enable,
    bool remove_error,
    int16_t speed_rpm_setpoint,
    int16_t positive_torque_limit,
    int16_t negative_torque_limit
) 
{
    _inverter_setpoints.inverter_enable = inverter_enable;
    _inverter_setpoints.hv_enable = hv_enable;
    _inverter_setpoints.driver_enable = driver_enable;
    _inverter_setpoints.remove_error = remove_error;
    _inverter_setpoints.speed_rpm_setpoint = speed_rpm_setpoint;
    _inverter_setpoints.positive_torque_limit = positive_torque_limit;
    _inverter_setpoints.negative_torque_limit = negative_torque_limit;
}

void InverterInterface::set_torque_command(uint16_t torque_command) 
{
    _inverter_torque_command.torque_command = torque_command;
}


/**
 * Recieving CAN messages
 */
void InverterInterface::recieve_MC_ENERGY(CAN_message_t &can_msg) 
{
    // Unpack the message
    MC1_ENERGY_t unpacked_msg;
    Unpack_MC1_ENERGY_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _inverter_status.dc_bus_voltage = unpacked_msg.dc_bus_voltage;
    _feedback_data.motor_mechanics.actual_torque = unpacked_msg.feedback_torque;
    _feedback_data.motor_mechanics.actual_power = unpacked_msg.motor_power;

}

void InverterInterface::recieve_MC_STATUS(CAN_message_t &can_msg) 
{

    // Unpack the message
    MC1_STATUS_t unpacked_msg;
    Unpack_MC1_STATUS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    
    // Update inverter interface with new data
    _inverter_status.system_ready = unpacked_msg.system_ready;
    _inverter_status.error = unpacked_msg.error;
    _inverter_status.warning = unpacked_msg.warning;
    _inverter_status.quit_dc_on = unpacked_msg.quit_dc_on;
    _inverter_status.dc_on = unpacked_msg.dc_on;
    _inverter_status.quit_inverter_on = unpacked_msg.quit_inverter_on;
    _inverter_status.derating_on = unpacked_msg.derating_on;

}

void InverterInterface::recieve_MC_TEMPS(CAN_message_t &can_msg)
{

    // Unpack the message
    MC1_TEMPS_t unpacked_msg;
    Unpack_MC1_TEMPS_hytech(&unpacked_msg, can_msg.buf, can_msg.len);

    // Update inverter interface with new data
    _inverter_status.diagnostic_number = unpacked_msg.diagnostic_number;
    _inverter_temps.igbt_temp = HYTECH_igbt_temp_ro_fromS(unpacked_msg.igbt_temp_ro);
    _inverter_temps.inverter_temp = HYTECH_inverter_temp_ro_fromS(unpacked_msg.inverter_temp_ro);
    _inverter_temps.motor_temp = HYTECH_motor_temp_ro_fromS(unpacked_msg.motor_temp_ro);

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

    msg_out.inverter_enable = _inverter_setpoints.inverter_enable;
    msg_out.hv_enable = _inverter_setpoints.hv_enable;
    msg_out.driver_enable = _inverter_setpoints.driver_enable;
    msg_out.remove_error = _inverter_setpoints.remove_error;
    msg_out.speed_setpoint_rpm = _inverter_setpoints.speed_rpm_setpoint;
    msg_out.positive_torque_limit_ro = _inverter_setpoints.positive_torque_limit;
    msg_out.negative_torque_limit_ro = _inverter_setpoints.negative_torque_limit;

    enqueue_new_CAN<MC1_SETPOINTS_COMMAND_t>(&msg_out, &Pack_MC1_SETPOINTS_COMMAND_hytech, inverter_ids.mc_setpoint_commands_id);
}

void InverterInterface::send_MC_TORQUE_COMMAND() 
{
    MC1_TORQUE_COMMAND_t msg_out;

    msg_out.torque_command_ro = HYTECH_torque_command_ro_toS(_inverter_torque_command.torque_command);

    enqueue_new_CAN<MC1_TORQUE_COMMAND_t>(&msg_out, &Pack_MC1_TORQUE_COMMAND_hytech, inverter_ids.mc_torque_command_id);
}

