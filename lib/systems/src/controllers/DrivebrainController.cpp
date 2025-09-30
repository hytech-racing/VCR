#include "controllers/DrivebrainController.h"
#include "SharedFirmwareTypes.h"
#include <cstdint>

// #include <iostream>

DrivetrainCommand_s DrivebrainController::evaluate(const VCRData_s &state, unsigned long curr_millis)
{

    auto db_input = state.interface_data.latest_drivebrain_command;
    
    // cases for timing_failure:
    // 1 we have not received any messages from the db (timestamped message recvd flag initialized as false in struct def)
    bool not_all_messages_recvd = ( (!db_input.desired_speeds.recvd) || (!db_input.torque_limits.recvd) );
    
    auto last_speed_setpoint_timestamp = db_input.desired_speeds.last_recv_millis;
    auto last_torque_lim_timestamp = db_input.torque_limits.last_recv_millis;

    // 2 if the time between the current VCR curr_millis time and the last millis time that we recvd a drivebrain msg is too high
    bool speed_setpoint_msg_too_latent = (::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_speed_setpoint_timestamp))) > (int)_params.allowed_latency);
    bool torque_limit_message_too_latent = (::abs((int)(static_cast<int64_t>(curr_millis) - static_cast<int64_t>(last_torque_lim_timestamp))) > (int)_params.allowed_latency);

    // std::cout << "last_speedsp ts " << last_speed_setpoint_timestamp <<std::endl;
    // std::cout << "ts " << last_torque_lim_timestamp <<std::endl;
    // 3 if the relative latency is too high (time between the message members) -> (allowed latency / 2)
    bool latency_diff_too_high = (::abs((int)(static_cast<int64_t>(last_speed_setpoint_timestamp) - static_cast<int64_t>(last_torque_lim_timestamp))) > ((int)_params.allowed_latency / 2));
    
    constexpr int64_t debug_timestamp_period_ms = 5000;

    if((static_cast<int64_t>(curr_millis) - static_cast<int64_t>(_last_worst_latency_timestamp)) > debug_timestamp_period_ms)
    {    
        _last_worst_latency_timestamp = curr_millis;
        _worst_message_latencies = {-1, -1};
    }

    if( (curr_millis - last_speed_setpoint_timestamp) > _worst_message_latencies.worst_speed_setpoint_latency_so_far)
    {
        _worst_message_latencies.worst_speed_setpoint_latency_so_far = (curr_millis - last_speed_setpoint_timestamp);   
    } else if((curr_millis - last_torque_lim_timestamp) > _worst_message_latencies.worst_torque_lim_latency_so_far)
    {
        _worst_message_latencies.worst_torque_lim_latency_so_far = (curr_millis - last_torque_lim_timestamp);   
    }

    bool timing_failure = (speed_setpoint_msg_too_latent || torque_limit_message_too_latent || not_all_messages_recvd || latency_diff_too_high);
    // if(timing_failure)
    // {
    //     std::cout <<"timing failures: "<<std::endl;
    //     std::cout << speed_setpoint_msg_too_latent<< std::endl;
    //     std::cout << torque_limit_message_too_latent<< std::endl;
    //     std::cout << not_all_messages_recvd<< std::endl;
    //     std::cout << latency_diff_too_high<< std::endl;
    // }
    // only if this is being evaluated while not the active control mode do we clear fault
    bool is_active_controller = state.system_data.tc_mux_status.active_controller_mode == _params.assigned_controller_mode;

    if ((!is_active_controller) && (!timing_failure))
    {
        // timing failure should be false here
        _timing_failure = false;
    }

    DrivetrainCommand_s output;
    if (!timing_failure && (!_timing_failure))
    {   
        output = db_input.get_command();
    }
    else
    {
        _timing_failure = true;
        DrivetrainCommand_s coast_to_stop = {
            .desired_speeds = {0.0f, 0.0f, 0.0f, 0.0f},
            .torque_limits = {0.0f, 0.0f, 0.0f, 0.0f}
        };
        output = coast_to_stop;
    }
    return output;
}