#include "controllers/DrivebrainController.h"
#include "SharedFirmwareTypes.h"
#include <cstdint>
#include <Arduino.h>
// #include <Arduino.h>

DrivetrainCommand_s DrivebrainController::evaluate(const VCRData_s &state, unsigned long curr_millis) {

    auto db_telem_input = state.interface_data.latest_drivebrain_telem_command;
    auto db_auxillary_input = state.interface_data.latest_drivebrain_auxillary_command;

    _telem_timing_failure = _check_drivebrain_command_timing_failure(db_telem_input, curr_millis);
    _aux_timing_failure = _check_drivebrain_command_timing_failure(db_auxillary_input, curr_millis);

    bool drivebrain_reinit_button_pressed = state.interface_data.dash_input_state.data_btn_is_pressed;

    if (drivebrain_reinit_button_pressed && (!_should_run_controller)) {
        _should_run_controller = true;
    }

    DrivetrainCommand_s output;

    if (_should_run_controller && !_telem_timing_failure) {
        output = db_telem_input.get_command();
    } else if (_should_run_controller && !_aux_timing_failure) {
        output = db_auxillary_input.get_command();
    } else {
        _should_run_controller = false;
        output = _emergency_control.evaluate(state, curr_millis);
    }

    return output;
}

bool DrivebrainController::_check_drivebrain_command_timing_failure(StampedDrivetrainCommand_s command, unsigned long curr_millis) {
    // Cases for timing_failure:

    // 1. we have not received any messages from the db (timestamped message recvd flag initialized as false in struct def)
    bool not_all_messages_recvd = ((!command.desired_speeds.recvd) || (!command.torque_limits.recvd));
    
    // 2. if the time between the current VCR curr_millis time and the last millis time that we recvd a drivebrain msg is too high
    auto last_speed_setpoint_timestamp = command.desired_speeds.last_recv_millis;
    auto last_torque_lim_timestamp = command.torque_limits.last_recv_millis;

    int speed_setpoint_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_speed_setpoint_timestamp)));
    int torque_setpoint_latency = ::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_torque_lim_timestamp)));

    bool speed_setpoint_msg_too_latent = (speed_setpoint_latency > (int)_params.allowed_latency);
    bool torque_limit_message_too_latent = (torque_setpoint_latency > (int)_params.allowed_latency);

    // 3. if the relative latency is too high (time between the message members) -> (allowed latency / 2)
    int relative_latency = ::abs((int)(static_cast<int64_t>(last_speed_setpoint_timestamp) - static_cast<int64_t>(last_torque_lim_timestamp)));
    bool latency_diff_too_high = (relative_latency > ((int)_params.allowed_latency / 2));
        
    if ((curr_millis - last_speed_setpoint_timestamp) > _worst_message_latencies.worst_speed_setpoint_latency_so_far) {
        _worst_message_latencies.worst_speed_setpoint_latency_so_far = static_cast<int64_t>(curr_millis - last_speed_setpoint_timestamp);   
    } else if ((curr_millis - last_torque_lim_timestamp) > _worst_message_latencies.worst_torque_lim_latency_so_far) {
        _worst_message_latencies.worst_torque_lim_latency_so_far = static_cast<int64_t>(curr_millis - last_torque_lim_timestamp);   
    }

    bool timing_failure = (speed_setpoint_msg_too_latent || torque_limit_message_too_latent || not_all_messages_recvd || latency_diff_too_high);
    return timing_failure;
}

