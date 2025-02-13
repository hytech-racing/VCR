#include "controllers/DrivebrainController.h"
#include "SharedFirmwareTypes.h"
#include <cstdint>

DrivetrainCommand_s DrivebrainController::evaluate(const VCRData_s &state, unsigned long eval_millis)
{


    
    auto db_input = state.interface_data.latest_drivebrain_command;
    
    // cases for timing_failure:
    // 1 we have not received any messages from the db (timestamped message recvd flag initialized as false in struct def)
    bool no_messages_received = (!db_input.recvd);
    
    
    // 2 if the time between the current VCR eval_millis time and the last millis time that we recvd a drivebrain msg is too high
    bool message_too_latent = (::abs((int)(static_cast<int64_t>(eval_millis) - static_cast<int64_t>(db_input.last_recv_millis))) > (int)_params.allowed_latency);
    constexpr int64_t debug_timestamp_period_ms = 5000;
    if((static_cast<int64_t>(eval_millis) - static_cast<int64_t>(_last_worst_latency_timestamp)) > debug_timestamp_period_ms)
    {    
        _last_worst_latency_timestamp = eval_millis;
        _worst_latency_so_far = -1;
    }

    if( (eval_millis - db_input.last_recv_millis) > _worst_latency_so_far)
    {
        _worst_latency_so_far = (eval_millis - db_input.last_recv_millis);   
    }
    

    bool timing_failure = (message_too_latent || no_messages_received);

    // only in the case that our speed is low enough (<1 m/s) do we want to clear the fault
    
    bool is_active_controller = state.system_data.tc_mux_status.active_controller_mode == _params.assigned_controller_mode;

    if ((!is_active_controller) && (!timing_failure))
    {
        // timing failure should be false here
        _timing_failure = false;
    }

    DrivetrainCommand_s output;
    if (!timing_failure && (!_timing_failure))
    {
        _last_setpoint_millis = db_input.last_recv_millis;
        
        output = db_input.cmd_data;
    }
    else
    {
        _timing_failure = true;
        // output.command = {{0.0f}, {0.0f}};
        output = _emergency_control.evaluate(state, eval_millis);
    }
    return output;
}